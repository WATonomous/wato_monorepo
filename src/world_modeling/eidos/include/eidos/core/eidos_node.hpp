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

#include "eidos/core/estimator.hpp"
#include "eidos/core/init_sequencer.hpp"
#include "eidos/core/map_manager.hpp"
#include "eidos/core/plugin_registry.hpp"
#include "eidos/utils/types.hpp"
#include "eidos_msgs/msg/slam_status.hpp"
#include "eidos_msgs/srv/load_map.hpp"
#include "eidos_msgs/srv/save_map.hpp"

namespace eidos
{

/**
 * @brief Top-level SLAM/localization node.
 *
 * Pure SLAM — handles the factor graph, optimization, and map persistence.
 * Does NOT broadcast TF or manage transforms (that's eidos_transform's job).
 *
 * Owns:
 * - InitSequencer: state machine (INIT → WARMUP → RELOCALIZING → TRACKING)
 * - Estimator: ISAM2 optimization engine
 * - MapManager: keyframe + data store
 * - PluginRegistry: owns all factor/relocalization/visualization plugins
 *
 * Threading:
 * - SLAM loop: own callback group, slam_rate_ Hz. All plugin reads lock-free.
 * - Each vis plugin: own callback group + timer.
 * - Plugin sensor callbacks: each plugin has its own callback group.
 */
class EidosNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EidosNode(const rclcpp::NodeOptions & options);
  ~EidosNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // ---- SLAM loop ----
  void tick();
  void handleTracking(double timestamp);
  void beginTracking(const gtsam::Pose3 & initial_pose);

  // ---- Publishing ----
  std::optional<gtsam::Pose3> getCurrentPose() const;
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
  Estimator estimator_;
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
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;
  std::string map_load_directory_;
  std::string map_save_directory_;
  std::vector<double> odom_pose_cov_;
};

}  // namespace eidos
