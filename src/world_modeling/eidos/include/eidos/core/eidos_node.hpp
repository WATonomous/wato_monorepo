// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "eidos/core/graph_optimizer.hpp"
#include "eidos/core/init_sequencer.hpp"
#include "eidos/map/map_manager.hpp"
#include "eidos/plugins/plugin_registry.hpp"
#include "eidos/utils/types.hpp"
#include "eidos_msgs/msg/slam_status.hpp"
#include "eidos_msgs/srv/load_map.hpp"
#include "eidos_msgs/srv/save_map.hpp"

namespace eidos
{

/// @brief A pending new state collected from a produceFactor() call.
struct NewState
{
  gtsam::Key key;  ///< GTSAM key allocated for this state.
  double ts;  ///< Sensor timestamp of the new state.
  StampedFactorResult result;  ///< Factors and initial values from the plugin.
  std::string owner;  ///< Name of the plugin that created this state.
};

/**
 * @brief Top-level SLAM/localization node.
 *
 * Owns:
 * - InitSequencer: state machine (INIT → WARMUP → RELOCALIZING → TRACKING)
 * - GraphOptimizer: ISAM2 optimization engine
 * - MapManager: keyframe + data store
 * - PluginRegistry: owns all factor/relocalization/visualization plugins
 *
 * Threading:
 * - SLAM loop: own callback group, slam_rate_ Hz. All plugin reads lock-free.
 * - Each vis plugin: own callback group + timer.
 */
class EidosNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct the EidosNode in an unconfigured state.
   * @param options ROS 2 node options forwarded to LifecycleNode.
   */
  explicit EidosNode(const rclcpp::NodeOptions & options);

  /// @brief Destructor.
  ~EidosNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configure the node: declare/read parameters, set up GraphOptimizer,
   *        load plugins, create publishers and services.
   * @param state Current lifecycle state (unused).
   * @return SUCCESS on successful configuration.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node: load prior map (if configured), activate
   *        publishers and plugins, and start the SLAM timer.
   * @param state Current lifecycle state (unused).
   * @return SUCCESS on successful activation.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node: cancel the SLAM timer, deactivate plugins
   *        and publishers.
   * @param state Current lifecycle state (unused).
   * @return SUCCESS on successful deactivation.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Clean up the node: reset timer and TF buffer.
   * @param state Current lifecycle state (unused).
   * @return SUCCESS on successful cleanup.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Handle shutdown of the node.
   * @param state Current lifecycle state (unused).
   * @return SUCCESS.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // ---- SLAM loop ----

  /**
   * @brief Main SLAM loop callback, invoked at slam_rate_ Hz.
   *
   * Delegates to InitSequencer while not yet TRACKING. Once TRACKING,
   * calls handleTracking() and publishes status.
   */
  void tick();

  /**
   * @brief Execute one tracking iteration: poll factor plugins for new
   *        factors/states, optimize the graph, update keyframes, and publish.
   * @param timestamp Current wall-clock time in seconds.
   */
  void handleTracking(double timestamp);

  /**
   * @brief Transition into the TRACKING state with the given initial pose.
   *
   * Resets the GraphOptimizer, sets state to TRACKING, and notifies all factor
   * plugins via onTrackingBegin().
   * @param initial_pose The initial pose at which tracking begins.
   */
  void beginTracking(const gtsam::Pose3 & initial_pose);

  // ---- Publishing ----

  /**
   * @brief Get the current best pose estimate.
   *
   * Tries the GraphOptimizer's optimized pose first; falls back to querying
   * factor plugins for a map pose (e.g. during relocalization).
   * @return The current pose, or std::nullopt if no pose is available yet.
   */
  std::optional<gtsam::Pose3> getCurrentPose() const;

  /// @brief Publish a SlamStatus message with current graph/plugin stats.
  void publishStatus();

  /// @brief Publish the current optimized pose as a PoseStamped in the map frame.
  void publishPose();

  /**
   * @brief Publish the current optimized pose as an Odometry message.
   * @note Diagonal covariance is populated from the odom_pose_cov_ parameter.
   */
  void publishOdometry();

  // ---- Services ----

  /**
   * @brief Service callback to save the current map to disk.
   * @param request Contains an optional filepath; falls back to map_save_directory_.
   * @param response Populated with success flag and message.
   */
  void saveMapCallback(
    const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response);

  /**
   * @brief Service callback to load a map from disk.
   * @param request Contains the filepath to load from.
   * @param response Populated with success flag and message.
   */
  void loadMapCallback(
    const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response);

  // ---- Core components ----
  GraphOptimizer graph_optimizer_;
  InitSequencer init_sequencer_;
  MapManager map_manager_;
  PluginRegistry registry_;

  // ---- TF buffer (shared with plugins for extrinsic lookups) ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- State ----
  std::atomic<SlamState> state_{SlamState::INITIALIZING};

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

  // ---- Cross-plugin state bridging ----
  gtsam::Key last_state_key_{0};
  double last_state_ts_{0.0};
  std::string last_state_owner_;
  bool has_last_state_{false};

  // ---- Parameters ----
  double slam_rate_;
  int update_iterations_;
  int correction_iterations_;
  int loop_closure_iterations_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string map_load_directory_;
  std::string map_save_directory_;
  std::vector<double> odom_pose_cov_;
};

}  // namespace eidos
