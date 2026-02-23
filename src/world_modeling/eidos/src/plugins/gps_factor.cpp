#include "eidos/plugins/gps_factor.hpp"

#include <cmath>
#include <fstream>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "eidos/slam_core.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GpsFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".gps_topic", "gps/odometry");
  node_->declare_parameter(prefix + ".cov_threshold", 2.0);
  node_->declare_parameter(prefix + ".use_elevation", false);
  node_->declare_parameter(prefix + ".min_trajectory_length", 5.0);
  node_->declare_parameter(prefix + ".gps_time_tolerance", 0.2);
  node_->declare_parameter(prefix + ".min_gps_movement", 5.0);
  node_->declare_parameter(prefix + ".no_elevation_noise", 0.01);
  node_->declare_parameter(prefix + ".min_noise_variance", 1.0);

  // ---- Read parameters ----
  std::string gps_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".cov_threshold", cov_threshold_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_trajectory_length", min_trajectory_length_);
  node_->get_parameter(prefix + ".gps_time_tolerance", gps_time_tolerance_);
  node_->get_parameter(prefix + ".min_gps_movement", min_gps_movement_);
  node_->get_parameter(prefix + ".no_elevation_noise", no_elevation_noise_);
  node_->get_parameter(prefix + ".min_noise_variance", min_noise_variance_);

  // ---- Create subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsFactor::gpsCallback, this, std::placeholders::_1),
      sub_opts);

  // ---- Register keyframe data types with MapManager ----
  auto& map_manager = core_->getMapManager();
  map_manager.registerType("gps_factor/position", {
    [](const std::any& data, const std::string& path) {
      auto pos = std::any_cast<gtsam::Point3>(data);
      std::ofstream ofs(path, std::ios::binary);
      double vals[3] = {pos.x(), pos.y(), pos.z()};
      ofs.write(reinterpret_cast<const char*>(vals), 3 * sizeof(double));
    },
    [](const std::string& path) -> std::any {
      std::ifstream ifs(path, std::ios::binary);
      double vals[3];
      ifs.read(reinterpret_cast<char*>(vals), 3 * sizeof(double));
      return gtsam::Point3(vals[0], vals[1], vals[2]);
    }
  });

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void GpsFactor::activate() {
  active_ = true;
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
      nav_msgs::msg::Odometry this_gps = gps_queue_.front();
      gps_queue_.pop_front();

      // Check GPS covariance - skip noisy measurements
      float noise_x = static_cast<float>(this_gps.pose.covariance[0]);
      float noise_y = static_cast<float>(this_gps.pose.covariance[7]);
      float noise_z = static_cast<float>(this_gps.pose.covariance[14]);
      if (noise_x > cov_threshold_ || noise_y > cov_threshold_) continue;

      float gps_x = static_cast<float>(this_gps.pose.pose.position.x);
      float gps_y = static_cast<float>(this_gps.pose.pose.position.y);
      float gps_z = static_cast<float>(this_gps.pose.pose.position.z);

      if (!use_elevation_) {
        gps_z = static_cast<float>(state_pose.translation().z());
        noise_z = no_elevation_noise_;
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
      noise_vec << std::max(static_cast<double>(noise_x), min_noise_variance_),
          std::max(static_cast<double>(noise_y), min_noise_variance_),
          std::max(static_cast<double>(noise_z), min_noise_variance_);
      auto gps_noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);

      gtsam::Point3 gps_position(gps_x, gps_y, gps_z);

      auto gps_factor = gtsam::make_shared<gtsam::GPSFactor>(
          state_index, gps_position, gps_noise);
      factors.push_back(gps_factor);

      // Store GPS position in MapManager
      core_->getMapManager().addKeyframeData(
          state_index, "gps_factor/position", gps_position);

      RCLCPP_DEBUG(node_->get_logger(),
                   "[%s] Added GPS factor at state %d (%.1f, %.1f, %.1f)",
                   name_.c_str(), state_index, gps_x, gps_y, gps_z);
      break;  // One GPS factor per state
    }
  }

  return factors;
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void GpsFactor::gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
