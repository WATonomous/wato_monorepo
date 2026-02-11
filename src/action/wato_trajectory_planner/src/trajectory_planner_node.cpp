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

#include "wato_trajectory_planner/trajectory_planner_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace wato_trajectory_planner
{

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("trajectory_planner_node", options)
{
  declare_parameter("safe_distance", 10.0);
  declare_parameter("stop_distance", 2.0);
  declare_parameter("max_speed", 5.0);
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  TrajectoryConfig config;
  config.safe_distance = get_parameter("safe_distance").as_double();
  config.stop_distance = get_parameter("stop_distance").as_double();
  config.max_speed = get_parameter("max_speed").as_double();

  core_ = std::make_unique<TrajectoryCore>(config);

  traj_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>("trajectory", 10);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "input_path", 10, std::bind(&TrajectoryPlannerNode::path_callback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 10, std::bind(&TrajectoryPlannerNode::costmap_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  traj_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  traj_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  traj_pub_.reset();
  path_sub_.reset();
  costmap_sub_.reset();
  core_.reset();
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void TrajectoryPlannerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  latest_path_ = msg;
  update_trajectory();
}

void TrajectoryPlannerNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = msg;
  // Costmap updates might trigger re-planning in the future
}

void TrajectoryPlannerNode::update_trajectory()
{
  if (!latest_path_ || !latest_costmap_ || !core_) {
    return;
  }
  auto traj = core_->compute_trajectory(*latest_path_, *latest_costmap_);
  traj_pub_->publish(traj);
}

}  // namespace wato_trajectory_planner

RCLCPP_COMPONENTS_REGISTER_NODE(wato_trajectory_planner::TrajectoryPlannerNode)
