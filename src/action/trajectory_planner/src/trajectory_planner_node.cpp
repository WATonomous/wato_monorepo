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

#include "trajectory_planner/trajectory_planner_node.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace trajectory_planner
{

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("trajectory_planner_node", options)
{
  RCLCPP_INFO(get_logger(), "Trajectory Planner lifecycle node created");

  // Publisher Topic Names
  declare_parameter("traj_pub_topic", "trajectory");
  declare_parameter("marker_pub_topic", "trajectory_markers");

  // Subscriber Topic Names
  declare_parameter("path_sub_topic", "input_path");
  declare_parameter("costmap_sub_topic", "costmap");
  declare_parameter("lane_context_sub_topic", "lane_context");
  declare_parameter("odom_sub_topic", "odom");
  declare_parameter("bt_sub_topic", "execute_behaviour");

  // Obstacle avoidance distances
  declare_parameter("stop_distance", 2.0);

  // Speed and path resolution
  declare_parameter("max_speed", 20.0);
  declare_parameter("max_tangential_accel", 2.0);
  declare_parameter("max_emergency_accel", 5.0);
  declare_parameter("max_lateral_accel", 2.0);
  declare_parameter("interpolation_resolution", 0.1);

  // Vehicle footprint bounding box for costmap collision checks
  declare_parameter("footprint_frame", std::string("base_link"));
  declare_parameter("footprint_x_min", -0.5);
  declare_parameter("footprint_y_min", -1.2);
  declare_parameter("footprint_x_max", 3.5);
  declare_parameter("footprint_y_max", 1.2);
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Trajectory Planner node");

  // Publisher Topic Names
  traj_pub_topic = get_parameter("traj_pub_topic").as_string();
  marker_pub_topic = get_parameter("marker_pub_topic").as_string();

  // Subscriber Topic Names
  path_sub_topic = get_parameter("path_sub_topic").as_string();
  costmap_sub_topic = get_parameter("costmap_sub_topic").as_string();
  lane_context_sub_topic = get_parameter("lane_context_sub_topic").as_string();
  odom_sub_topic = get_parameter("odom_sub_topic").as_string();
  bt_sub_topic = get_parameter("bt_sub_topic").as_string();

  // Build config from declared parameters and construct the planning core
  TrajectoryConfig config;
  config.stop_distance = get_parameter("stop_distance").as_double();
  config.max_speed = get_parameter("max_speed").as_double();
  config.max_tangential_accel = get_parameter("max_tangential_accel").as_double();
  config.max_emergency_accel = get_parameter("max_emergency_accel").as_double();
  config.max_lateral_accel = get_parameter("max_lateral_accel").as_double();
  config.interpolation_resolution = get_parameter("interpolation_resolution").as_double();
  config.footprint_x_min = get_parameter("footprint_x_min").as_double();
  config.footprint_y_min = get_parameter("footprint_y_min").as_double();
  config.footprint_x_max = get_parameter("footprint_x_max").as_double();
  config.footprint_y_max = get_parameter("footprint_y_max").as_double();
  // footprint_frame stored for documentation; bounding box is defined in vehicle body frame
  // aligned with path pose orientation at runtime
  std::string footprint_frame = get_parameter("footprint_frame").as_string();
  (void)footprint_frame;  // used as configuration documentation; TF lookup deferred

  core_ = std::make_unique<TrajectoryCore>(config);

  // Publishers — activated separately in on_activate
  traj_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>(traj_pub_topic, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_pub_topic, 10);

  // Subscribers
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_sub_topic, 10, std::bind(&TrajectoryPlannerNode::path_callback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_sub_topic, 10, std::bind(&TrajectoryPlannerNode::costmap_callback, this, std::placeholders::_1));

  lane_context_sub_ = create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    lane_context_sub_topic, 10, std::bind(&TrajectoryPlannerNode::lane_context_callback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_sub_topic, 10, std::bind(&TrajectoryPlannerNode::odom_callback, this, std::placeholders::_1));

  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(
    bt_sub_topic, 10, std::bind(&TrajectoryPlannerNode::bt_callback, this, std::placeholders::_1));

  // TF listener for cross-frame path transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Trajectory Planner node configured successfully");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating Trajectory Planner node");
  traj_pub_->on_activate();
  marker_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Trajectory Planner node activated");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Trajectory Planner node");
  traj_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Trajectory Planner node deactivated");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Trajectory Planner node");
  traj_pub_.reset();
  marker_pub_.reset();
  path_sub_.reset();
  costmap_sub_.reset();
  lane_context_sub_.reset();
  odom_sub_.reset();
  bt_sub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  RCLCPP_INFO(get_logger(), "Trajectory Planner node cleaned up");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down Trajectory Planner node");
  return on_cleanup(state);
}

void TrajectoryPlannerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  latest_path_ = msg;
}

void TrajectoryPlannerNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = msg;
  update_trajectory();
}

void TrajectoryPlannerNode::lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  // Update speed limit from the current lanelet; replaces the config max_speed cap
  current_speed_limit_mps_ = msg->current_lanelet.speed_limit_mps;
  has_speed_limit_ = true;
}

void TrajectoryPlannerNode::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  current_speed_mps = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
}

void TrajectoryPlannerNode::bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg)
{
  bt_requested_behaviour = msg->behaviour;
}

void TrajectoryPlannerNode::update_trajectory()
{
  // Wait until all inputs are ready before computing
  if (!latest_path_ || !latest_costmap_ || !core_ || current_speed_mps < 0.0 || bt_requested_behaviour.empty()) {
    return;
  }

  // Transform path to costmap frame if the frames differ
  nav_msgs::msg::Path transformed_path = *latest_path_;
  if (latest_path_->header.frame_id != latest_costmap_->header.frame_id) {
    try {
      // Check if transform is available before attempting lookup
      if (!tf_buffer_->canTransform(
            latest_costmap_->header.frame_id, latest_path_->header.frame_id, tf2::TimePointZero))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Cannot transform path from %s to %s",
          latest_path_->header.frame_id.c_str(),
          latest_costmap_->header.frame_id.c_str());
        return;
      }

      // Transform path
      transformed_path.header.frame_id = latest_costmap_->header.frame_id;
      transformed_path.poses.clear();

      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        latest_costmap_->header.frame_id, latest_path_->header.frame_id, tf2::TimePointZero);

      for (const auto & pose : latest_path_->poses) {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, transform);
        transformed_path.poses.push_back(transformed_pose);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
      return;
    }
  }

  // Use lane speed limit if available, otherwise fall back to config max_speed
  double limit_speed = has_speed_limit_ ? current_speed_limit_mps_ : get_parameter("max_speed").as_double();
  if (bt_requested_behaviour == "standby") limit_speed = 0.0;
  auto traj = core_->compute_trajectory(transformed_path, *latest_costmap_, limit_speed, current_speed_mps);

  // Publish trajectory in the original path frame (map) so it doesn't drift with the ego frame
  traj.header.frame_id = latest_path_->header.frame_id;
  for (size_t i = 0; i < traj.points.size() && i < latest_path_->poses.size(); ++i) {
    traj.points[i].pose = latest_path_->poses[i].pose;
  }

  // Publish trajectory
  traj_pub_->publish(traj);

  // Visualization
  if (marker_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray markers;

    // Delete all previous markers first
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    // Create a sphere marker for each point
    int id = 0;
    for (const auto & point : traj.points) {
      visualization_msgs::msg::Marker sphere;
      sphere.header = traj.header;
      sphere.ns = "trajectory_velocity";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = point.pose;

      // Size correlates with speed
      // Base size 0.1m, scales up to 0.5m at max speed
      double speed_ratio = std::max(0.0, std::min(1.0, point.max_speed / limit_speed));
      double diameter = 0.1 + (0.4 * speed_ratio);

      // Only add labels for every 4th point to avoid clutter
      if (id % 4 == 0) {
        visualization_msgs::msg::Marker label;
        label.header = traj.header;
        label.ns = "trajectory_speed_labels";
        label.id = id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose = point.pose;
        label.pose.position.y += 0.8;
        label.pose.position.z += 0.5;
        label.scale.z = 0.4;
        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;

        // Simple concatenation: use std::to_string and truncate extra zeros
        std::string speed_str = std::to_string(point.max_speed);
        label.text = speed_str.substr(0, speed_str.find(".") + 2) + " m/s";
        markers.markers.push_back(label);
      }

      sphere.scale.x = diameter;
      sphere.scale.y = diameter;
      sphere.scale.z = diameter;

      // Uniform color (Purple)
      sphere.color.r = 0.6f;
      sphere.color.g = 0.2f;
      sphere.color.b = 0.8f;
      sphere.color.a = 0.8f;

      markers.markers.push_back(sphere);
    }

    marker_pub_->publish(markers);
  }
}

}  // namespace trajectory_planner

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_planner::TrajectoryPlannerNode)
