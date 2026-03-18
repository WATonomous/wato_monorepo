#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "eidos/utils/types.hpp"
#include "eidos/core/estimator.hpp"
#include "eidos/core/init_sequencer.hpp"
#include "eidos/core/map_manager.hpp"
#include "eidos/core/plugin_registry.hpp"
#include "eidos/core/transform_manager.hpp"

#include "eidos_msgs/msg/slam_status.hpp"
#include "eidos_msgs/srv/save_map.hpp"
#include "eidos_msgs/srv/load_map.hpp"

namespace eidos {

/**
 * @brief Top-level SLAM/localization node.
 *
 * Owns all components:
 * - InitSequencer: state machine (INIT → WARMUP → RELOCALIZING → TRACKING)
 * - Estimator: ISAM2 optimization engine
 * - TransformManager: autonomous TF broadcaster (own timer)
 * - MapManager: keyframe + data store
 * - PluginRegistry: owns all plugins + ClassLoaders
 *
 * The SLAM loop dispatches to InitSequencer until TRACKING is reached,
 * then runs handleSlamTracking / handleLocalizationTracking.
 *
 * Autonomous components (EidosNode never calls them at runtime):
 * - TransformManager: own timer, reads poses lock-free, broadcasts TF.
 * - Visualization plugins: each has own timer, reads values lock-free.
 *
 * Threading:
 * - SLAM loop: own callback group, slam_rate_ Hz. All plugin reads lock-free.
 * - TransformManager: own callback group + timer.
 * - Each vis plugin: own callback group + timer.
 * - Plugin sensor callbacks: each plugin has its own callback group.
 */
class EidosNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit EidosNode(const rclcpp::NodeOptions& options);
  ~EidosNode() override;

protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  // ---- SLAM loop ----

  /// Main tick at slam_rate_ Hz. Delegates to InitSequencer until TRACKING,
  /// then runs the appropriate tracking handler.
  void tick();

  /// TRACKING (SLAM mode) — polls factor plugins for factors, feeds them
  /// to Estimator, runs ISAM2 optimization, notifies plugins of corrections.
  void handleSlamTracking(double timestamp);

  /// TRACKING (localization mode) — polls factor plugins for pose updates.
  /// No ISAM2 optimization. TransformManager handles TF autonomously.
  void handleLocalizationTracking(double timestamp);

  /// Called by InitSequencer when TRACKING is reached. Resets Estimator
  /// and notifies all factor plugins via onTrackingBegin().
  void beginTracking(const gtsam::Pose3& initial_pose);

  // ---- Publishing ----
  void publishStatus();
  void publishPose();
  void publishOdometry();

  // ---- Services ----
  void saveMapCallback(
      const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
      std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response);
  void loadMapCallback(
      const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
      std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response);

  // ---- Core components ----
  Estimator estimator_;                ///< ISAM2 optimization engine
  InitSequencer init_sequencer_;       ///< State machine (INIT → TRACKING)
  TransformManager transform_manager_; ///< Autonomous TF broadcaster (own timer)
  MapManager map_manager_;             ///< Keyframe + global data store
  PluginRegistry registry_;            ///< Owns all plugins + ClassLoaders

  // ---- TF buffer (shared with plugins for extrinsic lookups) ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- State ----
  std::atomic<SlamState> state_{SlamState::INITIALIZING}; ///< Lock-free, read by plugins
  std::string mode_;  ///< "slam" or "localization", from config

  // ---- SLAM timer ----
  rclcpp::TimerBase::SharedPtr slam_timer_;
  rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::SlamStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ---- Services ----
  rclcpp::Service<eidos_msgs::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<eidos_msgs::srv::LoadMap>::SharedPtr load_map_srv_;

  // ---- Parameters (all set from config in on_configure, no header defaults) ----
  double slam_rate_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string map_load_directory_;
  std::string map_save_directory_;
  std::vector<double> odom_pose_cov_;  ///< Published odometry covariance diagonal
};

}  // namespace eidos
