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

#ifndef WATO_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_
#define WATO_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_

#include <memory>

#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"
#include "wato_trajectory_planner/trajectory_core.hpp"

namespace wato_trajectory_planner
{

class TrajectoryPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);
  void update_trajectory();

  std::unique_ptr<TrajectoryCore> core_;
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr lane_context_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::SharedPtr latest_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  /// Current lane speed limit in m/s. Falls back to config max_speed if unavailable.
  double current_speed_limit_mps_{0.0};
  bool has_speed_limit_{false};
};

}  // namespace wato_trajectory_planner

#endif  // WATO_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_
