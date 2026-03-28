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

#include <memory>
#include <string>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_planner/trajectory_core.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace trajectory_planner
{

class TrajectoryPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Subscription callbacks
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);

  // Recomputes and publishes trajectory when new path or costmap arrives
  void update_trajectory();

  // Planning core — stateless, holds trajectory config
  std::unique_ptr<TrajectoryCore> core_;

  // Publisher Topic Names
  std::string traj_pub_topic, marker_pub_topic;

  // Subscriber Topic Names
  std::string path_sub_topic, costmap_sub_topic, lane_context_sub_topic, odom_sub_topic, bt_sub_topic;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr lane_context_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_;

  // TF — used to transform path into costmap frame when frames differ
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached latest inputs from upstream nodes
  nav_msgs::msg::Path::SharedPtr latest_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::string bt_requested_behaviour;

  // Speed limit from lane context; falls back to config max_speed if unavailable
  double current_speed_limit_mps_{0.0};
  bool has_speed_limit_{false};

  // Current speed of car
  double current_speed_mps{-1.0};
};

}  // namespace trajectory_planner
