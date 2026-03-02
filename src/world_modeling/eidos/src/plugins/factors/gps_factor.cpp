#include "eidos/plugins/factors/gps_factor.hpp"

#include <cmath>
#include <fstream>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam_unstable/slam/BiasedGPSFactor.h>
#include <gtsam/slam/PriorFactor.h>
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

  node_->declare_parameter(prefix + ".bias_prior_sigma", 50.0);

  // ---- Read parameters ----
  std::string gps_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".cov_threshold", cov_threshold_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_trajectory_length", min_trajectory_length_);
  node_->get_parameter(prefix + ".gps_time_tolerance", gps_time_tolerance_);
  node_->get_parameter(prefix + ".min_gps_movement", min_gps_movement_);
  node_->get_parameter(prefix + ".bias_prior_sigma", bias_prior_sigma_);

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

  // ---- Publisher for raw GPS in UTM frame ----
  utm_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      name_ + "/utm_pose", 10);

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

  // Global data: bias (3 doubles) + zone info (1 double)
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

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (BiasedGPSFactor mode)", name_.c_str());
}

void GpsFactor::activate() {
  active_ = true;
  utm_pub_->on_activate();

  // Check if a prior bias was loaded from a saved map
  auto& map_manager = core_->getMapManager();
  auto global = map_manager.getGlobalData("gps_factor/utm_to_map");
  if (global.has_value()) {
    auto offset = std::any_cast<Eigen::Vector4d>(global.value());
    latest_bias_ = gtsam::Point3(offset[0], offset[1], offset[2]);
    // Decode zone info: zone * 10 + (is_north ? 1 : 0)
    int encoded = static_cast<int>(offset[3]);
    utm_zone_ = encoded / 10;
    utm_is_north_ = (encoded % 10) != 0;
    // Don't set bias_initialized_ — let getFactors re-add the prior+value
    // to the new ISAM2 graph, using the loaded bias as initial value.
    broadcastUtmToMap();
    RCLCPP_INFO(node_->get_logger(), "[%s] loaded prior bias (zone %d%c): [%.1f, %.1f, %.1f]",
                name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S',
                latest_bias_.x(), latest_bias_.y(), latest_bias_.z());
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
  bias_initialized_ = false;
  latest_bias_ = gtsam::Point3(0, 0, 0);
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
// getFactors - add BiasedGPSFactor constraint if available and reliable
// ---------------------------------------------------------------------------
FactorResult GpsFactor::getFactors(
    int state_index, const gtsam::Pose3& state_pose, double /*timestamp*/) {
  FactorResult result;

  if (!active_) return result;

  std::lock_guard<std::mutex> lock(gps_lock_);

  if (gps_queue_.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] getFactors: gps_queue_ empty", name_.c_str());
    return result;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] getFactors: state=%d, queue_size=%zu",
              name_.c_str(), state_index, gps_queue_.size());

  // For state > 0, apply trajectory-length gate
  if (state_index > 0) {
    auto key_poses_3d = core_->getMapManager().getKeyPoses3D();
    if (key_poses_3d->size() > 1) {
      float trajectory_length = pointDistance(
          key_poses_3d->points.front(), key_poses_3d->points.back());
      if (trajectory_length < min_trajectory_length_) {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] getFactors: trajectory too short (%.2f < %.2f m)",
                    name_.c_str(), trajectory_length, min_trajectory_length_);
        return result;
      }
    }
  }

  // Get the timestamp of the current state
  double state_time = node_->now().seconds();
  auto poses_6d = core_->getMapManager().getKeyPoses6D();
  if (!poses_6d->empty()) {
    state_time = static_cast<double>(poses_6d->points.back().time);
  }

  RCLCPP_INFO(node_->get_logger(),
              "[%s] getFactors: matching GPS to state_time=%.3f, queue front=%.3f, back=%.3f",
              name_.c_str(), state_time,
              stamp2Sec(gps_queue_.front().header.stamp),
              stamp2Sec(gps_queue_.back().header.stamp));

  // Find GPS message closest to the state time
  int dropped_old = 0, skipped_no_fix = 0, skipped_cov = 0, skipped_movement = 0;
  float last_movement_dist = 0.0f;
  bool added = false;

  while (!gps_queue_.empty()) {
    if (stamp2Sec(gps_queue_.front().header.stamp) < state_time - gps_time_tolerance_) {
      gps_queue_.pop_front();
      ++dropped_old;
    } else if (stamp2Sec(gps_queue_.front().header.stamp) > state_time + gps_time_tolerance_) {
      break;
    } else {
      sensor_msgs::msg::NavSatFix this_fix = gps_queue_.front();
      gps_queue_.pop_front();

      // Filter by status — skip STATUS_NO_FIX
      if (this_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        ++skipped_no_fix;
        continue;
      }

      // Extract covariance from NavSatFix
      double noise_x = this_fix.position_covariance[0];  // east variance
      double noise_y = this_fix.position_covariance[4];  // north variance
      double noise_z = this_fix.position_covariance[8];  // up variance

      if (noise_x > cov_threshold_ || noise_y > cov_threshold_) {
        ++skipped_cov;
        continue;
      }

      // Convert lat/lon to UTM
      UtmCoordinate utm = latLonToUtm(
          this_fix.latitude, this_fix.longitude, this_fix.altitude);

      Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);

      // Initialize bias variable on first accepted fix
      if (!bias_initialized_) {
        utm_zone_ = utm.zone;
        utm_is_north_ = utm.is_north;

        // Compute initial bias so error ≈ 0: bias = utm_pos - map_pos
        Eigen::Vector3d map_pos = state_pose.translation();
        latest_bias_ = gtsam::Point3(
            utm_pos.x() - map_pos.x(),
            utm_pos.y() - map_pos.y(),
            utm_pos.z() - map_pos.z());

        auto bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, bias_prior_sigma_);
        result.factors.push_back(
            gtsam::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
                bias_key_, latest_bias_, bias_prior_noise));
        result.values.insert(bias_key_, latest_bias_);

        bias_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(),
                     "[%s] Initialized bias variable (zone %d%c), initial=[%.1f, %.1f, %.1f]",
                     name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S',
                     latest_bias_.x(), latest_bias_.y(), latest_bias_.z());
      }

      // Prepare GPS measurement in UTM coordinates
      double gps_z = utm_pos.z();
      if (!use_elevation_) {
        // Neutralize elevation: set measured z so error in z ≈ 0
        gps_z = state_pose.translation().z() + latest_bias_.z();
        noise_z = 1e6;
      }
      gtsam::Point3 gps_measurement(utm_pos.x(), utm_pos.y(), gps_z);

      // Skip if GPS hasn't moved enough from last injection (in raw UTM)
      float gps_x_utm = static_cast<float>(utm_pos.x());
      float gps_y_utm = static_cast<float>(utm_pos.y());
      if (has_last_gps_) {
        float dx = gps_x_utm - last_gps_point_.x;
        float dy = gps_y_utm - last_gps_point_.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_gps_movement_) {
          ++skipped_movement;
          last_movement_dist = dist;
          continue;
        }
      }

      last_gps_point_.x = gps_x_utm;
      last_gps_point_.y = gps_y_utm;
      last_gps_point_.z = static_cast<float>(utm_pos.z());
      has_last_gps_ = true;

      // Create BiasedGPSFactor: error = pose.translation() + bias - measured
      gtsam::Vector3 noise_vec;
      noise_vec << noise_x, noise_y, noise_z;
      auto gps_noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);

      auto gps_factor = gtsam::make_shared<gtsam::BiasedGPSFactor>(
          state_index, bias_key_, gps_measurement, gps_noise);
      result.factors.push_back(gps_factor);

      // Store UTM position in MapManager
      double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
      Eigen::Vector4d utm_data(utm.easting, utm.northing, utm.altitude, zone_encoded);
      core_->getMapManager().addKeyframeData(
          state_index, "gps_factor/utm_position", utm_data);

      RCLCPP_INFO(node_->get_logger(),
                  "\033[35m[%s] ADDED BiasedGPSFactor at state %d: "
                  "utm=(%.2f, %.2f, %.2f) noise=(%.3f, %.3f, %.3f) lat=%.7f lon=%.7f\033[0m",
                  name_.c_str(), state_index,
                  utm_pos.x(), utm_pos.y(), utm_pos.z(),
                  noise_x, noise_y, noise_z,
                  this_fix.latitude, this_fix.longitude);
      added = true;
      break;  // One GPS factor per state
    }
  }

  if (!added) {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] getFactors: no factor added (dropped_old=%d, no_fix=%d, "
                "cov_reject=%d, movement_reject=%d, max_dist=%.2f/%.2f, queue_remaining=%zu)",
                name_.c_str(), dropped_old, skipped_no_fix, skipped_cov,
                skipped_movement, last_movement_dist, min_gps_movement_,
                gps_queue_.size());
  }

  return result;
}

// ---------------------------------------------------------------------------
// onOptimizationComplete — read optimized bias, broadcast TF, persist
// ---------------------------------------------------------------------------
void GpsFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  if (!bias_initialized_) return;
  if (!optimized_values.exists(bias_key_)) return;

  // Read the optimized bias
  gtsam::Point3 prev_bias = latest_bias_;
  latest_bias_ = optimized_values.at<gtsam::Point3>(bias_key_);

  RCLCPP_INFO(node_->get_logger(),
              "[%s] bias: [%.3f, %.3f, %.3f] delta: [%.3f, %.3f, %.3f]",
              name_.c_str(),
              latest_bias_.x(), latest_bias_.y(), latest_bias_.z(),
              latest_bias_.x() - prev_bias.x(),
              latest_bias_.y() - prev_bias.y(),
              latest_bias_.z() - prev_bias.z());

  // Broadcast TF
  broadcastUtmToMap();

  // Persist bias + zone info for map save/load
  auto& map_manager = core_->getMapManager();
  double zone_encoded = static_cast<double>(utm_zone_ * 10 + (utm_is_north_ ? 1 : 0));
  Eigen::Vector4d global_offset(
      latest_bias_.x(), latest_bias_.y(), latest_bias_.z(), zone_encoded);
  map_manager.setGlobalData("gps_factor/utm_to_map", global_offset);
}

// ---------------------------------------------------------------------------
// broadcastUtmToMap — publish static TF using the optimized bias
//
// bias = utm_pos - map_pos  (by BiasedGPSFactor definition)
// So: map_pos = utm_pos - bias
// The TF utm → map is: translation = -bias (to go from utm to map, subtract bias)
// ---------------------------------------------------------------------------
void GpsFactor::broadcastUtmToMap() {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = utm_frame_;
  tf.child_frame_id = map_frame_;
  tf.transform.translation.x = -latest_bias_.x();
  tf.transform.translation.y = -latest_bias_.y();
  tf.transform.translation.z = -latest_bias_.z();
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
    RCLCPP_INFO(node_->get_logger(),
                "[%s] first NavSatFix received: lat=%.7f lon=%.7f status=%d cov=[%.3f, %.3f, %.3f]",
                name_.c_str(), msg->latitude, msg->longitude, msg->status.status,
                msg->position_covariance[0], msg->position_covariance[4],
                msg->position_covariance[8]);
  }

  // Publish raw GPS position in UTM frame
  if (active_) {
    UtmCoordinate utm = latLonToUtm(msg->latitude, msg->longitude, msg->altitude);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = utm_frame_;
    pose.pose.position.x = utm.easting;
    pose.pose.position.y = utm.northing;
    pose.pose.position.z = utm.altitude;
    pose.pose.orientation.w = 1.0;
    utm_pub_->publish(pose);
  }
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
