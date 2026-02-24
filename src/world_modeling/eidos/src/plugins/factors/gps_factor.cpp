#include "eidos/plugins/factors/gps_factor.hpp"

#include <cmath>
#include <fstream>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/navigation/GPSFactor.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GpsFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".gps_topic", "gps/fix");
  node_->declare_parameter(prefix + ".cov_threshold", 2.0);
  node_->declare_parameter(prefix + ".use_elevation", false);
  node_->declare_parameter(prefix + ".min_trajectory_length", 5.0);
  node_->declare_parameter(prefix + ".gps_time_tolerance", 0.2);
  node_->declare_parameter(prefix + ".min_gps_movement", 5.0);
  node_->declare_parameter(prefix + ".min_noise_variance", 1.0);

  // ---- Read parameters ----
  std::string gps_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".cov_threshold", cov_threshold_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_trajectory_length", min_trajectory_length_);
  node_->get_parameter(prefix + ".gps_time_tolerance", gps_time_tolerance_);
  node_->get_parameter(prefix + ".min_gps_movement", min_gps_movement_);
  node_->get_parameter(prefix + ".min_noise_variance", min_noise_variance_);

  // Read frame names from slam_core parameters
  node_->get_parameter("frames.map", map_frame_);

  // ---- Create subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::gpsCallback, this, std::placeholders::_1),
      sub_opts);

  // ---- Static TF broadcaster for utm → map ----
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  // ---- Register keyframe data types with MapManager ----
  auto& map_manager = core_->getMapManager();

  // UTM position per keyframe: 4 doubles (easting, northing, altitude, zone+hemisphere)
  map_manager.registerType("gps_factor/utm_position", {
    [](const std::any& data, const std::string& path) {
      auto pos = std::any_cast<Eigen::Vector4d>(data);
      std::ofstream ofs(path, std::ios::binary);
      double vals[4] = {pos[0], pos[1], pos[2], pos[3]};
      ofs.write(reinterpret_cast<const char*>(vals), 4 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      double vals[4];
      ifs.read(reinterpret_cast<char*>(vals), 4 * sizeof(double));
      return Eigen::Vector4d(vals[0], vals[1], vals[2], vals[3]);
    }
  });

  // Global data: utm_to_map offset (3 doubles + zone info)
  map_manager.registerGlobalType("gps_factor/utm_to_map", {
    [](const std::any& data, const std::string& path) {
      auto offset = std::any_cast<Eigen::Vector4d>(data);
      std::ofstream ofs(path, std::ios::binary);
      double vals[4] = {offset[0], offset[1], offset[2], offset[3]};
      ofs.write(reinterpret_cast<const char*>(vals), 4 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      double vals[4];
      ifs.read(reinterpret_cast<char*>(vals), 4 * sizeof(double));
      return Eigen::Vector4d(vals[0], vals[1], vals[2], vals[3]);
    }
  });

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (NavSatFix mode)", name_.c_str());
}

void GpsFactor::activate() {
  active_ = true;

  // Check if a prior utm_to_map offset was loaded
  auto& map_manager = core_->getMapManager();
  auto global = map_manager.getGlobalData("gps_factor/utm_to_map");
  if (global.has_value()) {
    auto offset = std::any_cast<Eigen::Vector4d>(global.value());
    utm_to_map_translation_ = offset.head<3>();
    // Decode zone info: zone * 10 + (is_north ? 1 : 0)
    int encoded = static_cast<int>(offset[3]);
    utm_zone_ = encoded / 10;
    utm_is_north_ = (encoded % 10) != 0;
    utm_to_map_initialized_ = true;
    broadcastUtmToMap();
    RCLCPP_INFO(node_->get_logger(), "[%s] loaded prior utm→map offset (zone %d%c)",
                name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S');
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsFactor::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void GpsFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  {
    std::lock_guard<std::mutex> lock(gps_lock_);
    gps_queue_.clear();
  }
  has_last_gps_ = false;
  utm_to_map_initialized_ = false;
}

// ---------------------------------------------------------------------------
// isReady - ready when at least 1 valid NavSatFix received
// ---------------------------------------------------------------------------
bool GpsFactor::isReady() const {
  return gps_received_;
}

std::string GpsFactor::getReadyStatus() const {
  return gps_received_ ? "fix acquired" : "no NavSatFix received";
}

// ---------------------------------------------------------------------------
// processFrame - GPS does not provide a pose estimate
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> GpsFactor::processFrame(double /*timestamp*/) {
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// getFactors - add GPS constraint if available and reliable
// ---------------------------------------------------------------------------
std::vector<gtsam::NonlinearFactor::shared_ptr> GpsFactor::getFactors(
    int state_index, const gtsam::Pose3& state_pose, double /*timestamp*/) {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  if (!active_) return factors;

  std::lock_guard<std::mutex> lock(gps_lock_);

  if (gps_queue_.empty()) return factors;

  // Need enough keyframes to establish trajectory before injecting GPS
  auto key_poses_3d = core_->getMapManager().getKeyPoses3D();
  if (key_poses_3d->points.empty()) return factors;

  // Wait until the trajectory covers minimum distance before adding GPS
  if (key_poses_3d->size() > 1) {
    float trajectory_length = pointDistance(
        key_poses_3d->points.front(), key_poses_3d->points.back());
    if (trajectory_length < min_trajectory_length_) return factors;
  }

  // Get the timestamp of the current state (approximate from latest keyframe)
  double state_time = node_->now().seconds();
  auto poses_6d = core_->getMapManager().getKeyPoses6D();
  if (!poses_6d->empty()) {
    state_time = static_cast<double>(poses_6d->points.back().time);
  }

  // Find GPS message closest to the state time
  while (!gps_queue_.empty()) {
    if (stamp2Sec(gps_queue_.front().header.stamp) < state_time - gps_time_tolerance_) {
      gps_queue_.pop_front();
    } else if (stamp2Sec(gps_queue_.front().header.stamp) > state_time + gps_time_tolerance_) {
      break;
    } else {
      sensor_msgs::msg::NavSatFix this_fix = gps_queue_.front();
      gps_queue_.pop_front();

      // Filter by status — skip STATUS_NO_FIX
      if (this_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        continue;
      }

      // Extract covariance from NavSatFix
      // NavSatFix uses a 3x3 row-major covariance [0]=lat, [4]=lon, [8]=alt
      // in ENU: [0] ~ east, [4] ~ north, [8] ~ up (approximation for small areas)
      double noise_x = this_fix.position_covariance[0];  // east variance
      double noise_y = this_fix.position_covariance[4];  // north variance
      double noise_z = this_fix.position_covariance[8];  // up variance

      if (noise_x > cov_threshold_ || noise_y > cov_threshold_) continue;

      // Convert lat/lon to UTM
      UtmCoordinate utm = latLonToUtm(
          this_fix.latitude, this_fix.longitude, this_fix.altitude);

      // Initialize utm → map on first accepted fix
      if (!utm_to_map_initialized_) {
        Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);
        Eigen::Vector3d map_pos = state_pose.translation();
        utm_to_map_translation_ = utm_pos - map_pos;
        utm_zone_ = utm.zone;
        utm_is_north_ = utm.is_north;
        utm_to_map_initialized_ = true;
        broadcastUtmToMap();
        RCLCPP_INFO(node_->get_logger(),
                     "[%s] Initialized utm→map offset (zone %d%c): [%.1f, %.1f, %.1f]",
                     name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S',
                     utm_to_map_translation_.x(), utm_to_map_translation_.y(),
                     utm_to_map_translation_.z());
      }

      // Convert UTM to map frame
      Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);
      Eigen::Vector3d map_pos = utm_pos - utm_to_map_translation_;

      float gps_x = static_cast<float>(map_pos.x());
      float gps_y = static_cast<float>(map_pos.y());
      float gps_z = static_cast<float>(map_pos.z());

      if (!use_elevation_) {
        gps_z = static_cast<float>(state_pose.translation().z());
        noise_z = 0.01;
      }

      // Skip if GPS hasn't moved enough from last injection
      if (has_last_gps_) {
        if (std::abs(gps_x - last_gps_point_.x) < min_gps_movement_ &&
            std::abs(gps_y - last_gps_point_.y) < min_gps_movement_) {
          continue;
        }
      }

      last_gps_point_.x = gps_x;
      last_gps_point_.y = gps_y;
      last_gps_point_.z = gps_z;
      has_last_gps_ = true;

      // Create GPS factor with covariance-based noise
      gtsam::Vector3 noise_vec;
      noise_vec << std::max(noise_x, min_noise_variance_),
          std::max(noise_y, min_noise_variance_),
          std::max(noise_z, min_noise_variance_);
      auto gps_noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);

      gtsam::Point3 gps_position(gps_x, gps_y, gps_z);

      auto gps_factor = gtsam::make_shared<gtsam::GPSFactor>(
          state_index, gps_position, gps_noise);
      factors.push_back(gps_factor);

      // Store UTM position in MapManager (for later offset recomputation)
      // Encode zone+hemisphere as: zone * 10 + (is_north ? 1 : 0)
      double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
      Eigen::Vector4d utm_data(utm.easting, utm.northing, utm.altitude, zone_encoded);
      core_->getMapManager().addKeyframeData(
          state_index, "gps_factor/utm_position", utm_data);

      RCLCPP_DEBUG(node_->get_logger(),
                   "[%s] Added GPS factor at state %d (map: %.1f, %.1f, %.1f)",
                   name_.c_str(), state_index, gps_x, gps_y, gps_z);
      break;  // One GPS factor per state
    }
  }

  return factors;
}

// ---------------------------------------------------------------------------
// onOptimizationComplete — recompute utm → map offset
// ---------------------------------------------------------------------------
void GpsFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!utm_to_map_initialized_) return;

  auto& map_manager = core_->getMapManager();
  int num_keyframes = map_manager.numKeyframes();

  Eigen::Vector3d offset_sum = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;

  for (int i = 0; i < num_keyframes; i++) {
    auto utm_data = map_manager.getKeyframeData(i, "gps_factor/utm_position");
    if (!utm_data.has_value()) continue;

    auto utm_vec = std::any_cast<Eigen::Vector4d>(utm_data.value());
    Eigen::Vector3d utm_pos = utm_vec.head<3>();

    if (!optimized_values.exists(i)) continue;
    auto optimized_pose = optimized_values.at<gtsam::Pose3>(i);
    Eigen::Vector3d map_pos = optimized_pose.translation();

    Eigen::Vector3d offset = utm_pos - map_pos;
    offset_sum += offset;
    weight_sum += 1.0;
  }

  if (weight_sum > 0.0) {
    utm_to_map_translation_ = offset_sum / weight_sum;
    broadcastUtmToMap();

    // Persist in MapManager global data
    double zone_encoded = static_cast<double>(utm_zone_ * 10 + (utm_is_north_ ? 1 : 0));
    Eigen::Vector4d global_offset(
        utm_to_map_translation_.x(), utm_to_map_translation_.y(),
        utm_to_map_translation_.z(), zone_encoded);
    map_manager.setGlobalData("gps_factor/utm_to_map", global_offset);
  }
}

// ---------------------------------------------------------------------------
// broadcastUtmToMap — publish static TF for utm → map
// ---------------------------------------------------------------------------
void GpsFactor::broadcastUtmToMap() {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = utm_frame_;
  tf.child_frame_id = map_frame_;
  // utm → map: to go from utm to map, subtract the offset
  // So map_pos = utm_pos - offset  =>  T_utm_map = -offset (translation only)
  tf.transform.translation.x = -utm_to_map_translation_.x();
  tf.transform.translation.y = -utm_to_map_translation_.y();
  tf.transform.translation.z = -utm_to_map_translation_.z();
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  static_tf_broadcaster_->sendTransform(tf);
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void GpsFactor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
  if (!gps_received_) {
    gps_received_ = true;
  }
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
