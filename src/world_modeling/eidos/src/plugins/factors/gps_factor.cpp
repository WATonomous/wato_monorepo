#include "eidos/plugins/factors/gps_factor.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam_unstable/slam/BiasedGPSFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  node_->declare_parameter(prefix + ".min_gps_movement", 5.0);

  node_->declare_parameter(prefix + ".bias_prior_cov",
      std::vector<double>{10000.0, 10000.0, 10000.0});
  node_->declare_parameter(prefix + ".gps_cov", std::vector<double>{1.0, 1.0, 1.0});
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");

  // ---- Read parameters ----
  std::string gps_topic, imu_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".cov_threshold", cov_threshold_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_gps_movement", min_gps_movement_);
  node_->get_parameter(prefix + ".bias_prior_cov", bias_prior_cov_);
  node_->get_parameter(prefix + ".gps_cov", gps_cov_);

  // Read frame names from slam_core parameters
  node_->get_parameter("frames.map", map_frame_);

  // ---- Create subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::gpsCallback, this, std::placeholders::_1),
      sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::imuCallback, this, std::placeholders::_1),
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

  // Global data: bias (3 doubles) + zone info (1 double) + initial yaw (1 double)
  map_manager.registerGlobalType("gps_factor/utm_to_map", {
    [](const std::any& data, const std::string& path) {
      auto offset = std::any_cast<std::array<double, 5>>(data);
      std::ofstream ofs(path, std::ios::binary);
      ofs.write(reinterpret_cast<const char*>(offset.data()), 5 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      std::array<double, 5> vals;
      ifs.read(reinterpret_cast<char*>(vals.data()), 5 * sizeof(double));
      return vals;
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
    auto offset = std::any_cast<std::array<double, 5>>(global.value());
    latest_bias_ = gtsam::Point3(offset[0], offset[1], offset[2]);
    // Decode zone info: zone * 10 + (is_north ? 1 : 0)
    int encoded = static_cast<int>(offset[3]);
    utm_zone_ = encoded / 10;
    utm_is_north_ = (encoded % 10) != 0;
    // Reconstruct heading rotation from stored yaw
    initial_yaw_ = offset[4];
    double cy = std::cos(-initial_yaw_);
    double sy = std::sin(-initial_yaw_);
    R_map_enu_ << cy, -sy, 0,
                  sy,  cy, 0,
                   0,   0, 1;
    // Don't set bias_initialized_ — let getFactors re-add the prior+value
    // to the new ISAM2 graph, using the loaded bias as initial value.
    broadcastUtmToMap();
    RCLCPP_INFO(node_->get_logger(), "[%s] loaded prior bias (zone %d%c, yaw=%.3f): [%.1f, %.1f, %.1f]",
                name_.c_str(), utm_zone_, utm_is_north_ ? 'N' : 'S',
                initial_yaw_,
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
  has_imu_orientation_.store(false, std::memory_order_relaxed);
  initial_yaw_ = 0.0;
  R_map_enu_ = Eigen::Matrix3d::Identity();
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

  // Get the timestamp of the current state
  double state_time = node_->now().seconds();
  auto poses_6d = core_->getMapManager().getKeyPoses6D();
  if (!poses_6d->empty()) {
    state_time = static_cast<double>(poses_6d->points.back().time);
  }

  // Find GPS message closest to the state time
  // Scan the queue, pick the fix with the smallest |t - state_time|,
  // then drain everything up to and including it.
  int best_idx = -1;
  double best_dt = std::numeric_limits<double>::max();
  for (size_t i = 0; i < gps_queue_.size(); ++i) {
    double dt = std::abs(stamp2Sec(gps_queue_[i].header.stamp) - state_time);
    if (dt < best_dt) {
      best_dt = dt;
      best_idx = static_cast<int>(i);
    } else {
      // Queue is time-ordered, so once dt starts increasing we're past the closest
      break;
    }
  }

  if (best_idx < 0) return result;

  // Take the closest fix and drain everything before it
  sensor_msgs::msg::NavSatFix this_fix = gps_queue_[best_idx];
  for (int i = 0; i <= best_idx; ++i) gps_queue_.pop_front();

  // Process the closest fix
  {

      // Filter by status — skip STATUS_NO_FIX
      if (this_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        return result;
      }

      // Sensor-reported covariance (varies per fix with satellite geometry)
      double sensor_cov_x = this_fix.position_covariance[0];
      double sensor_cov_y = this_fix.position_covariance[4];
      double sensor_cov_z = this_fix.position_covariance[8];

      if (sensor_cov_x > cov_threshold_ || sensor_cov_y > cov_threshold_) {
        return result;
      }

      // Convert lat/lon to UTM
      UtmCoordinate utm = latLonToUtm(
          this_fix.latitude, this_fix.longitude, this_fix.altitude);

      Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);

      // Initialize bias variable on first accepted fix
      if (!bias_initialized_) {
        // Need IMU heading to align map frame with UTM
        if (!has_imu_orientation_.load(std::memory_order_relaxed)) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                               "[%s] waiting for IMU orientation before initializing GPS bias",
                               name_.c_str());
          return result;
        }

        utm_zone_ = utm.zone;
        utm_is_north_ = utm.is_north;

        // Capture IMU yaw and build rotation from ENU/UTM to map frame.
        // At initialization, map X-axis = vehicle forward, which is at angle
        // `yaw` from East in ENU. So R_map_enu = Rz(-yaw).
        {
          std::lock_guard<std::mutex> lock(imu_orientation_lock_);
          initial_yaw_ = latest_imu_yaw_;
        }
        double cy = std::cos(-initial_yaw_);
        double sy = std::sin(-initial_yaw_);
        R_map_enu_ << cy, -sy, 0,
                      sy,  cy, 0,
                       0,   0, 1;

        // Rotate UTM position into map frame, then compute bias
        Eigen::Vector3d utm_rotated = utmToMap(utm_pos);
        Eigen::Vector3d map_pos(state_pose.translation().x(),
                                state_pose.translation().y(),
                                state_pose.translation().z());
        latest_bias_ = gtsam::Point3(
            utm_rotated.x() - map_pos.x(),
            utm_rotated.y() - map_pos.y(),
            use_elevation_ ? (utm_rotated.z() - map_pos.z()) : 0.0);

        auto bias_prior_noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(3) << bias_prior_cov_[0], bias_prior_cov_[1], bias_prior_cov_[2]).finished());
        result.factors.push_back(
            gtsam::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
                bias_key_, latest_bias_, bias_prior_noise));
        result.values.insert(bias_key_, latest_bias_);

        bias_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(),
                     "[%s] bias initialized: yaw=%.1f deg, bias=(%.1f, %.1f, %.1f)",
                     name_.c_str(), initial_yaw_ * 180.0 / M_PI,
                     latest_bias_.x(), latest_bias_.y(), latest_bias_.z());
      }

      // Rotate UTM position into map frame for the BiasedGPSFactor
      Eigen::Vector3d gps_rotated = utmToMap(utm_pos);

      if (!use_elevation_) {
        // Flat ground: constrain z near map origin (z=0).
        // measurement.z = 0 + bias.z(≈0) => error.z = pose.z + bias.z ≈ pose.z
        // This prevents unconstrained z drift from IMU integration errors.
        gps_rotated.z() = 0.0;
      }
      gtsam::Point3 gps_measurement(gps_rotated.x(), gps_rotated.y(), gps_rotated.z());

      // Skip if GPS hasn't moved enough from last injection (in raw UTM)
      float gps_x_utm = static_cast<float>(utm_pos.x());
      float gps_y_utm = static_cast<float>(utm_pos.y());
      if (has_last_gps_) {
        float dx = gps_x_utm - last_gps_point_.x;
        float dy = gps_y_utm - last_gps_point_.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_gps_movement_) {
          return result;
        }
      }

      last_gps_point_.x = gps_x_utm;
      last_gps_point_.y = gps_y_utm;
      last_gps_point_.z = static_cast<float>(utm_pos.z());
      has_last_gps_ = true;

      // Create BiasedGPSFactor: error = pose.translation() + bias - measured
      // Use max(sensor_cov, gps_cov) per axis — sensor covariance reflects
      // satellite geometry quality, gps_cov acts as a minimum floor.
      // When use_elevation=false, z measurement is overridden to 0.0 so we
      // always use the configured gps_cov[2] for z in that case.
      double cov_x = std::max(sensor_cov_x, gps_cov_[0]);
      double cov_y = std::max(sensor_cov_y, gps_cov_[1]);
      double cov_z = use_elevation_ ? std::max(sensor_cov_z, gps_cov_[2]) : gps_cov_[2];
      auto gps_noise = gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(3) << cov_x, cov_y, cov_z).finished());

      auto gps_factor = gtsam::make_shared<gtsam::BiasedGPSFactor>(
          state_index, bias_key_, gps_measurement, gps_noise);
      result.factors.push_back(gps_factor);

      // Store UTM position in MapManager
      double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
      Eigen::Vector4d utm_data(utm.easting, utm.northing, utm.altitude, zone_encoded);
      core_->getMapManager().addKeyframeData(
          state_index, "gps_factor/utm_position", utm_data);

      RCLCPP_INFO(node_->get_logger(),
                  "\033[35m[%s] added BiasedGPSFactor at state %d: "
                  "measurement=(%.3f, %.3f, %.3f) cov=(%.4f, %.4f, %.4f)\033[0m",
                  name_.c_str(), state_index,
                  gps_measurement.x(), gps_measurement.y(), gps_measurement.z(),
                  cov_x, cov_y, cov_z);
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
  latest_bias_ = optimized_values.at<gtsam::Point3>(bias_key_);

  // Broadcast TF
  broadcastUtmToMap();

  // Persist bias + zone info + yaw for map save/load
  auto& map_manager = core_->getMapManager();
  double zone_encoded = static_cast<double>(utm_zone_ * 10 + (utm_is_north_ ? 1 : 0));
  std::array<double, 5> global_offset = {
      latest_bias_.x(), latest_bias_.y(), latest_bias_.z(), zone_encoded, initial_yaw_};
  map_manager.setGlobalData("gps_factor/utm_to_map", global_offset);
}

// ---------------------------------------------------------------------------
// broadcastUtmToMap — publish static TF using the optimized bias + heading
// ---------------------------------------------------------------------------
void GpsFactor::broadcastUtmToMap() {
  // bias = R_map_enu * utm_pos - map_pos  (BiasedGPSFactor definition)
  // So: map_pos = R_map_enu * utm_pos - bias
  //
  // TF convention: p_parent = R_tf * p_child + t_tf
  // parent = utm, child = map
  // p_utm = R_enu_map * p_map + R_enu_map * bias
  //
  // R_enu_map = R_map_enu^T = Rz(initial_yaw_)
  Eigen::Matrix3d R_enu_map = R_map_enu_.transpose();
  Eigen::Vector3d bias_vec(latest_bias_.x(), latest_bias_.y(), latest_bias_.z());
  Eigen::Vector3d t_tf = R_enu_map * bias_vec;
  Eigen::Quaterniond q_enu_map(R_enu_map);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = utm_frame_;
  tf.child_frame_id = map_frame_;
  tf.transform.translation.x = t_tf.x();
  tf.transform.translation.y = t_tf.y();
  tf.transform.translation.z = t_tf.z();
  tf.transform.rotation.x = q_enu_map.x();
  tf.transform.rotation.y = q_enu_map.y();
  tf.transform.rotation.z = q_enu_map.z();
  tf.transform.rotation.w = q_enu_map.w();

  static_tf_broadcaster_->sendTransform(tf);
}

// ---------------------------------------------------------------------------
// IMU callback — capture latest yaw for heading alignment
// ---------------------------------------------------------------------------
void GpsFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  if (q.length2() < 0.01) return;  // invalid quaternion

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  std::lock_guard<std::mutex> lock(imu_orientation_lock_);
  latest_imu_yaw_ = yaw;
  has_imu_orientation_.store(true, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// utmToMap — rotate UTM position into map frame using initial heading
// ---------------------------------------------------------------------------
Eigen::Vector3d GpsFactor::utmToMap(const Eigen::Vector3d& utm_pos) const {
  return R_map_enu_ * utm_pos;
}

// ---------------------------------------------------------------------------
// GPS callback
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
    // Include IMU orientation if available (ENU frame)
    {
      std::lock_guard<std::mutex> imu_lock(imu_orientation_lock_);
      tf2::Quaternion q;
      q.setRPY(0, 0, latest_imu_yaw_);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    }
    utm_pub_->publish(pose);
  }
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
