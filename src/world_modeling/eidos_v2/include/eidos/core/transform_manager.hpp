#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/geometry/Pose3.h>

#include "eidos/utils/lock_free_pose.hpp"
#include "eidos/utils/pose_ekf.hpp"
#include "eidos/core/plugin_registry.hpp"

namespace eidos {

/**
 * @brief Single authority on TF broadcasting.
 *
 * Runs its own timer at a configurable rate. On each tick:
 * 1. Reads motion model pose (500 Hz) + odom_source pose (~10 Hz) via lock-free getters.
 * 2. Fuses them through a PoseEKF for smooth odom→base (no jumps).
 *    - MM provides prediction at every tick.
 *    - odom_source provides measurement update when new data arrives.
 * 3. Reads map-frame pose from Estimator or factor plugin.
 * 4. Computes map→odom = map_pose * inv(odom→base).
 * 5. Broadcasts both TFs.
 *
 * If odom_source is empty (""), the EKF is bypassed and motion model drives
 * odom→base directly (pure dead-reckoning, corrections only via map→odom).
 *
 * Config (no defaults — must be set in YAML):
 * - odom_source: factor plugin name, or "" for motion-model-only
 * - map_source: "slam_core" (reads Estimator pose) or factor plugin name
 * - rate: timer frequency in Hz
 * - fusion.process_noise: [roll,pitch,yaw,x,y,z] per-step process noise variance
 * - fusion.measurement_noise: [roll,pitch,yaw,x,y,z] measurement noise variance
 */
class TransformManager {
public:
  void configure(
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      const PluginRegistry* registry,
      const LockFreePose* estimator_pose,
      const std::string& map_frame,
      const std::string& odom_frame,
      const std::string& base_link_frame,
      const std::string& odom_source_name,
      const std::string& map_source_name,
      double rate_hz);

  void activate();
  void deactivate();

private:
  void tick();

  /// Compute odom→base using EKF fusion (or raw MM if no odom_source).
  gtsam::Pose3 computeOdomToBase();

  void broadcastTf(const rclcpp::Time& stamp,
                   const std::string& frame_id,
                   const std::string& child_frame_id,
                   const gtsam::Pose3& pose);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  const PluginRegistry* registry_ = nullptr;
  const LockFreePose* estimator_pose_ = nullptr;

  /// Unified fused odometry publisher (odom frame → base_link)
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  gtsam::Pose3 cached_map_to_odom_;
  gtsam::Pose3 cached_odom_to_base_;
  gtsam::Pose3 last_map_pose_;              ///< For detecting when map_src changes
  bool has_last_map_pose_ = false;

  // ---- EKF fusion (when odom_source is set) ----
  PoseEKF odom_ekf_;                        ///< Fuses MM prediction + odom_source corrections
  gtsam::Pose3 last_mm_pose_;               ///< MM pose at last tick (for computing delta)
  bool has_last_mm_ = false;
  gtsam::Pose3 last_odom_source_pose_;      ///< For detecting new odom_source readings
  bool has_odom_source_reading_ = false;

  std::string odom_source_name_;
  std::string map_source_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
};

}  // namespace eidos
