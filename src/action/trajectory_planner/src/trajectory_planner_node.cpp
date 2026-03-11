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

  // Planner selection
  declare_parameter("use_elastic", true);

  // Obstacle avoidance distances (used by legacy TrajectoryCore)
  declare_parameter("safe_distance", 10.0);
  declare_parameter("stop_distance", 2.0);

  // Speed and path resolution
  declare_parameter("max_speed", 20.0);
  declare_parameter("interpolation_resolution", 0.1);

  // Vehicle footprint bounding box for costmap collision checks
  declare_parameter("footprint_frame", std::string("base_link"));
  declare_parameter("footprint_x_min", -0.5);
  declare_parameter("footprint_y_min", -1.2);
  declare_parameter("footprint_x_max", 3.5);
  declare_parameter("footprint_y_max", 1.2);

  // --- Elastic planner parameters ---
  declare_parameter("elastic.footprint_sample_res", 0.15);
  declare_parameter("elastic.k_r", 0.030);
  declare_parameter("elastic.k_s", 0.150);
  declare_parameter("elastic.max_step", 0.050);
  declare_parameter("elastic.probe_width", 0.60);
  declare_parameter("elastic.num_iterations", 20);
  declare_parameter("elastic.lane_width", 4.0);
  declare_parameter("elastic.vehicle_width", 2.4);
  declare_parameter("elastic.lane_margin", 0.10);
  declare_parameter("elastic.tau_rep", 10.0);
  declare_parameter("elastic.tau_gate", 65.0);
  declare_parameter("elastic.epsilon", 5.0);
  declare_parameter("elastic.sigmoid_mu", 40.0);
  declare_parameter("elastic.sigmoid_w", 20.0);
  declare_parameter("elastic.alpha", 0.08);
  declare_parameter("elastic.a_max", 3.0);
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Trajectory Planner node");

  use_elastic_ = get_parameter("use_elastic").as_bool();

  // Shared footprint / speed params
  double fp_x_min = get_parameter("footprint_x_min").as_double();
  double fp_y_min = get_parameter("footprint_y_min").as_double();
  double fp_x_max = get_parameter("footprint_x_max").as_double();
  double fp_y_max = get_parameter("footprint_y_max").as_double();
  double max_speed = get_parameter("max_speed").as_double();

  if (use_elastic_) {
    ElasticConfig ecfg;
    ecfg.footprint_x_min = fp_x_min;
    ecfg.footprint_y_min = fp_y_min;
    ecfg.footprint_x_max = fp_x_max;
    ecfg.footprint_y_max = fp_y_max;
    ecfg.max_speed = max_speed;
    ecfg.footprint_sample_res = get_parameter("elastic.footprint_sample_res").as_double();
    ecfg.k_r = get_parameter("elastic.k_r").as_double();
    ecfg.k_s = get_parameter("elastic.k_s").as_double();
    ecfg.max_step = get_parameter("elastic.max_step").as_double();
    ecfg.probe_width = get_parameter("elastic.probe_width").as_double();
    ecfg.num_iterations = get_parameter("elastic.num_iterations").as_int();
    ecfg.lane_width = get_parameter("elastic.lane_width").as_double();
    ecfg.vehicle_width = get_parameter("elastic.vehicle_width").as_double();
    ecfg.lane_margin = get_parameter("elastic.lane_margin").as_double();
    ecfg.tau_rep = get_parameter("elastic.tau_rep").as_double();
    ecfg.tau_gate = get_parameter("elastic.tau_gate").as_double();
    ecfg.epsilon = get_parameter("elastic.epsilon").as_double();
    ecfg.sigmoid_mu = get_parameter("elastic.sigmoid_mu").as_double();
    ecfg.sigmoid_w = get_parameter("elastic.sigmoid_w").as_double();
    ecfg.alpha = get_parameter("elastic.alpha").as_double();
    ecfg.a_max = get_parameter("elastic.a_max").as_double();
    elastic_core_ = std::make_unique<ElasticCore>(ecfg);
    RCLCPP_INFO(get_logger(), "Using elastic lateral deformation planner");
  } else {
    // Legacy longitudinal-only planner
    TrajectoryConfig config;
    config.safe_distance = get_parameter("safe_distance").as_double();
    config.stop_distance = get_parameter("stop_distance").as_double();
    config.max_speed = max_speed;
    config.interpolation_resolution = get_parameter("interpolation_resolution").as_double();
    config.footprint_x_min = fp_x_min;
    config.footprint_y_min = fp_y_min;
    config.footprint_x_max = fp_x_max;
    config.footprint_y_max = fp_y_max;
    core_ = std::make_unique<TrajectoryCore>(config);
    RCLCPP_INFO(get_logger(), "Using legacy longitudinal trajectory planner");
  }

  // Publishers — activated separately in on_activate
  traj_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>("trajectory", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);
  speed_label_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_speed_labels", 10);
  visualizer_ = std::make_unique<TrajectoryVisualizer>(marker_pub_, speed_label_pub_);

  // Subscribers
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "input_path", 10, std::bind(&TrajectoryPlannerNode::path_callback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 10, std::bind(&TrajectoryPlannerNode::costmap_callback, this, std::placeholders::_1));

  lane_context_sub_ = create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context", 10, std::bind(&TrajectoryPlannerNode::lane_context_callback, this, std::placeholders::_1));

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
  speed_label_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Trajectory Planner node activated");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Trajectory Planner node");
  traj_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  speed_label_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Trajectory Planner node deactivated");
  return CallbackReturn::SUCCESS;
}

TrajectoryPlannerNode::CallbackReturn TrajectoryPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Trajectory Planner node");
  visualizer_.reset();
  traj_pub_.reset();
  marker_pub_.reset();
  speed_label_pub_.reset();
  path_sub_.reset();
  costmap_sub_.reset();
  lane_context_sub_.reset();
  core_.reset();
  elastic_core_.reset();
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
  update_trajectory();
}

void TrajectoryPlannerNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Cache latest costmap; used on the next path callback to check for obstacles
  latest_costmap_ = msg;
  update_trajectory();
}

void TrajectoryPlannerNode::lane_context_callback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  // Update speed limit from the current lanelet; replaces the config max_speed cap
  current_speed_limit_mps_ = msg->current_lanelet.speed_limit_mps;
  has_speed_limit_ = true;
}

void TrajectoryPlannerNode::update_trajectory()
{
  // Wait until all inputs are ready before computing
  const bool ready = latest_path_ && latest_costmap_ && (use_elastic_ ? !!elastic_core_ : !!core_);
  if (!ready) {
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

  auto traj = use_elastic_ ? elastic_core_->compute_trajectory(transformed_path, *latest_costmap_, limit_speed)
                           : core_->compute_trajectory(transformed_path, *latest_costmap_, limit_speed);

  // Transform trajectory back to the original path frame (e.g. map) if computation
  // was done in the costmap frame (e.g. base_footprint)
  const std::string & path_frame = latest_path_->header.frame_id;
  const std::string & costmap_frame = latest_costmap_->header.frame_id;
  if (path_frame != costmap_frame) {
    try {
      geometry_msgs::msg::TransformStamped inv_transform =
        tf_buffer_->lookupTransform(path_frame, costmap_frame, tf2::TimePointZero);

      traj.header.frame_id = path_frame;
      for (auto & pt : traj.points) {
        geometry_msgs::msg::PoseStamped ps_in, ps_out;
        ps_in.header.frame_id = costmap_frame;
        ps_in.pose = pt.pose;
        tf2::doTransform(ps_in, ps_out, inv_transform);
        pt.pose = ps_out.pose;
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Back-transform error: %s", ex.what());
      return;
    }
  }

  // Publish trajectory
  traj_pub_->publish(traj);

  // Visualization
  visualizer_->publish(traj, limit_speed);
}

}  // namespace trajectory_planner

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_planner::TrajectoryPlannerNode)
