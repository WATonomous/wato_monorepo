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

#include "freeroam_planner/freeroam_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "freeroam_planner/astar.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace freeroam_planner
{

FreeroamPlannerNode::FreeroamPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("freeroam_planner_node", options)
{
  declare_parameter("goal_topic", "goal_point");
  declare_parameter("costmap_topic", "/world_modeling/costmap");
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("base_frame", "base_footprint");
  declare_parameter("max_speed", 5.0);
  declare_parameter("goal_tolerance", 1.0);
  declare_parameter("obstacle_threshold", 50);
  declare_parameter("inflation_radius_m_", 1.5);
  declare_parameter("allow_diagonal", true);
  declare_parameter("planning_rate_hz", 2.0);
}

FreeroamPlannerNode::CallbackReturn FreeroamPlannerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  goal_topic_ = get_parameter("goal_topic").as_string();
  costmap_topic_ = get_parameter("costmap_topic").as_string();
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  max_speed_ = get_parameter("max_speed").as_double();
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  obstacle_threshold_ = static_cast<int>(get_parameter("obstacle_threshold").as_int());
  allow_diagonal_ = get_parameter("allow_diagonal").as_bool();
  planning_rate_hz_ = get_parameter("planning_rate_hz").as_double();
  inflation_radius_m_ = get_parameter("inflation_radius_m_").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  trajectory_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>(trajectory_topic_, rclcpp::QoS(10));

  // REMOVE - for path vis
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));

  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic_, rclcpp::QoS(10), std::bind(&FreeroamPlannerNode::goalCallback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(10), std::bind(&FreeroamPlannerNode::costmapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured: planning at %.1f Hz", planning_rate_hz_);
  return CallbackReturn::SUCCESS;
}

FreeroamPlannerNode::CallbackReturn FreeroamPlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_pub_->on_activate();
  // REMOVE - for path vis
  path_pub_->on_activate();

  const auto period = std::chrono::duration<double>(1.0 / planning_rate_hz_);
  plan_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&FreeroamPlannerNode::planCallback, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

FreeroamPlannerNode::CallbackReturn FreeroamPlannerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  plan_timer_.reset();
  trajectory_pub_->on_deactivate();
  // REMOVE - for path vis
  path_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

FreeroamPlannerNode::CallbackReturn FreeroamPlannerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_pub_.reset();
  // REMOVE - for path vis
  path_pub_.reset();
  goal_sub_.reset();
  costmap_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  latest_goal_.reset();
  latest_costmap_.reset();

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

FreeroamPlannerNode::CallbackReturn FreeroamPlannerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  plan_timer_.reset();
  trajectory_pub_.reset();
  goal_sub_.reset();
  costmap_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // REMOVE - for path vis
  path_pub_.reset();

  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void FreeroamPlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  latest_goal_ = msg;
}

void FreeroamPlannerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = inflateOccupancyGrid(msg, inflation_radius_m_);
}

void FreeroamPlannerNode::planCallback()
{
  if (!latest_goal_ || !latest_costmap_) {
    return;
  }

  const auto & costmap = *latest_costmap_;
  const std::string & costmap_frame = costmap.header.frame_id;

  // Transform goal into costmap frame
  geometry_msgs::msg::PointStamped goal_in_costmap;
  try {
    goal_in_costmap = tf_buffer_->transform(*latest_goal_, costmap_frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Cannot transform goal to costmap frame: %s", ex.what());
    return;
  }

  // Vehicle position in costmap frame (transform base_frame origin into costmap frame)
  geometry_msgs::msg::PointStamped vehicle_origin;
  vehicle_origin.header.frame_id = base_frame_;
  vehicle_origin.header.stamp = costmap.header.stamp;
  vehicle_origin.point.x = 0.0;
  vehicle_origin.point.y = 0.0;
  vehicle_origin.point.z = 0.0;

  geometry_msgs::msg::PointStamped vehicle_in_costmap;
  try {
    vehicle_in_costmap = tf_buffer_->transform(vehicle_origin, costmap_frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Cannot transform vehicle to costmap frame: %s", ex.what());
    return;
  }

  double start_x = vehicle_in_costmap.point.x;
  double start_y = vehicle_in_costmap.point.y;
  double goal_x = goal_in_costmap.point.x;
  double goal_y = goal_in_costmap.point.y;

  // Check if already at goal
  double dist_to_goal = std::hypot(goal_x - start_x, goal_y - start_y);
  if (dist_to_goal < goal_tolerance_) {
    return;
  }

  auto path = astar(costmap, start_x, start_y, goal_x, goal_y, obstacle_threshold_, allow_diagonal_);

  if (path.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "A* found no path to goal");
    return;
  }

  // Build trajectory
  wato_trajectory_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.header.frame_id = costmap_frame;

  // REMOVE - for path vis
  nav_msgs::msg::Path vis_path;
  vis_path.header.stamp = now();
  vis_path.header.frame_id = costmap_frame;

  for (size_t i = 0; i < path.size(); ++i) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = path[i].first;
    pt.pose.position.y = path[i].second;
    pt.pose.position.z = 0.0;

    // REMOVE - for path vis
    geometry_msgs::msg::PoseStamped geo_pt;
    geo_pt.header.stamp = now();
    geo_pt.header.frame_id = costmap_frame;
    geo_pt.pose.position.x = path[i].first;
    geo_pt.pose.position.y = path[i].second;
    geo_pt.pose.position.z = 0.0;
    vis_path.poses.push_back(geo_pt);

    // Compute yaw from consecutive points
    double yaw = 0.0;
    if (i + 1 < path.size()) {
      yaw = std::atan2(path[i + 1].second - path[i].second, path[i + 1].first - path[i].first);
    } else if (i > 0) {
      yaw = std::atan2(path[i].second - path[i - 1].second, path[i].first - path[i - 1].first);
    }
    pt.pose.orientation.z = std::sin(yaw / 2.0);
    pt.pose.orientation.w = std::cos(yaw / 2.0);

    // Speed: max_speed for most points, 0 at goal
    pt.max_speed = (i + 1 < path.size()) ? max_speed_ : 0.0;

    trajectory.points.push_back(pt);
  }

  trajectory_pub_->publish(trajectory);

  // REMOVE - for path vis
  path_pub_->publish(vis_path);
}

nav_msgs::msg::OccupancyGrid::SharedPtr FreeroamPlannerNode::inflateOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & msg, double radius_m, bool treat_unknown_as_occupied_source)
{
  const int8_t inflated_value = static_cast<int8_t>(obstacle_threshold_);

  if (!msg) return nullptr;

  auto out = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);

  const auto w = static_cast<int>(out->info.width);
  const auto h = static_cast<int>(out->info.height);
  if (w <= 0 || h <= 0) return out;
  if (out->info.resolution <= 0.0f) return out;
  if (radius_m <= 0.0) return out;

  const int r_cells = static_cast<int>(std::ceil(radius_m / out->info.resolution));
  if (r_cells <= 0) return out;

  // Precompute circular offsets (dx, dy) within radius in grid cells
  std::vector<std::pair<int, int>> offsets;
  offsets.reserve((2 * r_cells + 1) * (2 * r_cells + 1));
  const int r2 = r_cells * r_cells;
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if (dx * dx + dy * dy <= r2) offsets.emplace_back(dx, dy);
    }
  }

  // Collect source obstacle cells first (faster than re-checking while writing)
  std::vector<std::pair<int, int>> sources;
  sources.reserve(1024);
  const auto & in = msg->data;
  if (static_cast<int>(in.size()) != w * h) return out;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      const int8_t v = in[idx];
      const bool is_occ = (v >= obstacle_threshold_) || (treat_unknown_as_occupied_source && v < 0);
      if (is_occ) sources.emplace_back(x, y);
    }
  }

  // Inflate
  for (const auto & [sx, sy] : sources) {
    for (const auto & [dx, dy] : offsets) {
      const int nx = sx + dx;
      const int ny = sy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const int nidx = nx + ny * w;

      // Make inflation monotonic: never reduce occupancy values
      out->data[nidx] = std::max(out->data[nidx], inflated_value);
    }
  }

  return out;
}

}  // namespace freeroam_planner
