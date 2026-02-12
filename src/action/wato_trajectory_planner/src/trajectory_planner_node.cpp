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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wato_trajectory_planner
{

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("trajectory_planner_node", options)
{
  declare_parameter("safe_distance", 10.0);
  declare_parameter("stop_distance", 2.0);
  declare_parameter("max_speed", 20.0);
  declare_parameter("interpolation_resolution", 0.1);
  declare_parameter("footprint_radius", 1.2);
  declare_parameter("vehicle_front_offset", 2.5);
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  TrajectoryConfig config;
  config.safe_distance = get_parameter("safe_distance").as_double();
  config.stop_distance = get_parameter("stop_distance").as_double();
  config.max_speed = get_parameter("max_speed").as_double();
  config.interpolation_resolution = get_parameter("interpolation_resolution").as_double();
  config.footprint_radius = get_parameter("footprint_radius").as_double();

  core_ = std::make_unique<TrajectoryCore>(config);

  traj_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>("trajectory", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "input_path", 10, std::bind(&TrajectoryPlannerNode::path_callback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 10, std::bind(&TrajectoryPlannerNode::costmap_callback, this, std::placeholders::_1));

  lane_context_sub_ = create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context", 10,
    std::bind(&TrajectoryPlannerNode::lane_context_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  traj_pub_->on_activate();
  marker_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  traj_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  traj_pub_.reset();
  marker_pub_.reset();
  path_sub_.reset();
  costmap_sub_.reset();
  lane_context_sub_.reset();
  core_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
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
}

void TrajectoryPlannerNode::lane_context_callback(
  const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  current_speed_limit_mps_ = msg->current_lanelet.speed_limit_mps;
  has_speed_limit_ = true;
}

void TrajectoryPlannerNode::update_trajectory()
{
  if (!latest_path_ || !latest_costmap_ || !core_) {
    return;
  }

  // Transform path to costmap frame if necessary
  nav_msgs::msg::Path transformed_path = *latest_path_;
  if (latest_path_->header.frame_id != latest_costmap_->header.frame_id) {
    try {
      // Check if transform is available
      if (!tf_buffer_->canTransform(
          latest_costmap_->header.frame_id,
          latest_path_->header.frame_id,
          tf2::TimePointZero))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Cannot transform path from %s to %s",
          latest_path_->header.frame_id.c_str(),
          latest_costmap_->header.frame_id.c_str());
        return;
      }

      // Transform path
      transformed_path.header.frame_id = latest_costmap_->header.frame_id;
      transformed_path.poses.clear();
      
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        latest_costmap_->header.frame_id,
        latest_path_->header.frame_id,
        tf2::TimePointZero);

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
  double limit_speed = has_speed_limit_
    ? current_speed_limit_mps_
    : get_parameter("max_speed").as_double();

  auto traj = core_->compute_trajectory(transformed_path, *latest_costmap_, limit_speed);
  
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

}  // namespace wato_trajectory_planner

RCLCPP_COMPONENTS_REGISTER_NODE(wato_trajectory_planner::TrajectoryPlannerNode)
