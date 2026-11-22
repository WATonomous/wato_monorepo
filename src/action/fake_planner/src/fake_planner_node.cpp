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

#include "fake_planner/fake_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fake_planner
{

// Parameters
FakePlannerNode::FakePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fake_planner_node", options)
, release_time_(now())
{
  declare_parameter("maneuver_file", std::string(""));
  declare_parameter("frame_id", "odom");
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("publish_behaviour", true);
  declare_parameter("behaviour", "lane_follow");
  declare_parameter("anchoring", "auto");
  declare_parameter("start_on_activate", false);
  declare_parameter("horizon_m", 35.0);
  declare_parameter("trail_m", 2.0);
  declare_parameter("search_ahead_m", 400.0);
  declare_parameter("respawn_jump_m", 5.0);
  declare_parameter("finish_distance_m", 3.0);
  declare_parameter("finish_speed_mps", 0.25);
  declare_parameter("max_path_deviation_m", 5.0);
  declare_parameter("release_ramp_s", 3.0);
}

// Lifecycle callbacks
FakePlannerNode::CallbackReturn FakePlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (!loadTrajectory()) {
    return CallbackReturn::FAILURE;
  }

  trajectory_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>("trajectory", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);
  if (publish_behaviour_) {
    behaviour_pub_ = create_publisher<behaviour_msgs::msg::ExecuteBehaviour>("execute_behaviour", 10);
  }

  const auto snapshot_qos = rclcpp::QoS(1).transient_local().reliable();
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", snapshot_qos);
  reference_pub_ = create_publisher<nav_msgs::msg::Path>("reference_path", snapshot_qos);

  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_trajectory",
    std::bind(&FakePlannerNode::startTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop_trajectory",
    std::bind(&FakePlannerNode::stopTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&FakePlannerNode::resetTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Retain private services and the existing session aliases.
  session_start_srv_ = create_service<std_srvs::srv::Trigger>(
    "start_trajectory",
    std::bind(&FakePlannerNode::startTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  session_stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "stop_trajectory",
    std::bind(&FakePlannerNode::stopTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  session_reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "reset", std::bind(&FakePlannerNode::resetTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&FakePlannerNode::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured fake planner");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  resetTrajectory();
  trajectory_pub_->on_activate();
  marker_pub_->on_activate();
  status_pub_->on_activate();
  reference_pub_->on_activate();
  publishState("stopped");
  if (behaviour_pub_) {
    behaviour_pub_->on_activate();
  }
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
  // Follow /clock so pause cannot refresh controller commands.
  timer_ = rclcpp::create_timer(
    get_node_base_interface(),
    get_node_timers_interface(),
    get_clock(),
    rclcpp::Duration(period),
    std::bind(&FakePlannerNode::publishTrajectory, this));
  RCLCPP_INFO(get_logger(), "Activated fake planner; start_on_activate=%s", start_on_activate_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  trajectory_pub_->on_deactivate();
  publishState("stopped");
  marker_pub_->on_deactivate();
  status_pub_->on_deactivate();
  reference_pub_->on_deactivate();
  if (behaviour_pub_) {
    behaviour_pub_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "Deactivated fake planner");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  odom_sub_.reset();
  start_srv_.reset();
  stop_srv_.reset();
  reset_srv_.reset();
  session_start_srv_.reset();
  session_stop_srv_.reset();
  session_reset_srv_.reset();
  trajectory_pub_.reset();
  behaviour_pub_.reset();
  marker_pub_.reset();
  status_pub_.reset();
  reference_pub_.reset();
  const std::lock_guard<std::mutex> lock(core_mutex_);
  core_.reset();
  anchored_ = have_pose_ = moved_ = finished_ = trajectory_started_ = false;
  RCLCPP_INFO(get_logger(), "Cleaned up fake planner");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

// Trajectory operations
bool FakePlannerNode::loadTrajectory()
{
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  publish_behaviour_ = get_parameter("publish_behaviour").as_bool();
  behaviour_ = get_parameter("behaviour").as_string();
  start_on_activate_ = get_parameter("start_on_activate").as_bool();
  respawn_jump_m_ = get_parameter("respawn_jump_m").as_double();
  finish_distance_m_ = get_parameter("finish_distance_m").as_double();
  finish_speed_mps_ = get_parameter("finish_speed_mps").as_double();
  max_path_deviation_m_ = get_parameter("max_path_deviation_m").as_double();
  release_ramp_s_ = get_parameter("release_ramp_s").as_double();
  const auto maneuver_file = get_parameter("maneuver_file").as_string();
  const auto anchoring = get_parameter("anchoring").as_string();

  FakePlannerConfig config;
  config.frame_id = get_parameter("frame_id").as_string();
  config.horizon_m = get_parameter("horizon_m").as_double();
  config.trail_m = get_parameter("trail_m").as_double();
  config.search_ahead_m = get_parameter("search_ahead_m").as_double();
  const std::pair<const char *, double> positive[] = {
    {"publish_rate_hz", publish_rate_hz_}, {"horizon_m", config.horizon_m}, {"search_ahead_m", config.search_ahead_m}};
  const std::pair<const char *, double> non_negative[] = {
    {"trail_m", config.trail_m},
    {"respawn_jump_m", respawn_jump_m_},
    {"finish_distance_m", finish_distance_m_},
    {"finish_speed_mps", finish_speed_mps_},
    {"max_path_deviation_m", max_path_deviation_m_},
    {"release_ramp_s", release_ramp_s_}};
  for (const auto & [name, value] : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_ERROR(get_logger(), "%s must be finite and > 0", name);
      return false;
    }
  }
  for (const auto & [name, value] : non_negative) {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(get_logger(), "%s must be finite and >= 0", name);
      return false;
    }
  }
  if (anchoring != "auto" && anchoring != "relative" && anchoring != "absolute") {
    RCLCPP_ERROR(get_logger(), "anchoring must be auto, relative or absolute");
    return false;
  }
  if (maneuver_file.empty()) {
    RCLCPP_ERROR(get_logger(), "maneuver_file must name a JSON file");
    return false;
  }

  core_ = std::make_unique<FakePlannerCore>(config);
  std::string error;
  if (!core_->loadManeuver(maneuver_file, error)) {
    RCLCPP_ERROR(get_logger(), "Cannot load '%s': %s", maneuver_file.c_str(), error.c_str());
    core_.reset();
    return false;
  }
  if (core_->waypointCount() < 2) {
    RCLCPP_ERROR(get_logger(), "Maneuver must contain at least two waypoints");
    core_.reset();
    return false;
  }
  anchor_to_first_pose_ = anchoring == "auto" ? !core_->hasAbsoluteStart() : anchoring == "relative";
  if (!anchor_to_first_pose_) {
    core_->anchor(0.0, 0.0, 0.0);
    if (!core_->closed()) {
      RCLCPP_WARN(get_logger(), "An absolute open maneuver needs the vehicle returned to its start before replay");
    }
  }
  if (anchoring == "relative" && core_->hasAbsoluteStart()) {
    RCLCPP_WARN(get_logger(), "Relative anchoring moves the maneuver away from its authored coordinates");
  }
  if (!core_->closed() && core_->pathLength() <= finish_distance_m_) {
    RCLCPP_WARN(get_logger(), "Maneuver is shorter than finish_distance_m; review the completion threshold");
  }
  resetTrajectory();
  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'", core_->waypointCount(), maneuver_file.c_str());
  return true;
}

void FakePlannerNode::resetTrajectory()
{
  if (anchor_to_first_pose_) {
    core_->clear();
  } else {
    core_->rewind();
  }
  anchored_ = !anchor_to_first_pose_;
  have_pose_ = moved_ = finished_ = trajectory_started_ = false;
  explicitly_stopped_ = false;
  publishState("stopped");
  veh_speed_ = 0.0;
  if (start_on_activate_) {
    releaseTrajectory();
  }
}

void FakePlannerNode::releaseTrajectory()
{
  if (!trajectory_started_) {
    release_time_ = now();
  }
  trajectory_started_ = true;
  explicitly_stopped_ = false;
}

double FakePlannerNode::releaseRampScale()
{
  if (release_ramp_s_ <= 0.0) {
    return 1.0;
  }
  const double elapsed = (now() - release_time_).seconds();
  if (elapsed < 0.0) {
    // A world reset can move the simulation clock backwards.
    release_time_ = now();
    return 0.0;
  }
  return std::min(1.0, elapsed / release_ramp_s_);
}

void FakePlannerNode::publishTrajectory()
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  if (!core_) {
    return;
  }
  publishReference();
  if (finished_) {
    publishMarkers(core_->window());
    return;
  }
  if (!trajectory_started_) {
    if (core_->ready() && have_pose_ && !explicitly_stopped_) {
      publishState("ready");
    }
    publishMarkers(core_->window());
    return;
  }
  if (!core_->ready() || !have_pose_) {
    return;
  }
  if (moved_ && core_->distanceToEnd() <= finish_distance_m_ && veh_speed_ <= finish_speed_mps_) {
    finished_ = true;
    trajectory_started_ = false;
    publishState("finished");
    publishMarkers(core_->window());
    RCLCPP_INFO(get_logger(), "Maneuver finished; reset before replay");
    return;
  }

  auto trajectory = core_->window();
  trajectory.header.stamp = now();
  const double scale = releaseRampScale();
  for (auto & point : trajectory.points) {
    point.max_speed *= scale;
  }
  trajectory_pub_->publish(trajectory);
  publishMarkers(trajectory);
  if (behaviour_pub_) {
    behaviour_msgs::msg::ExecuteBehaviour message;
    message.behaviour = behaviour_;
    behaviour_pub_->publish(message);
  }
}

// Subscription and service callbacks
void FakePlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  if (!core_) {
    return;
  }
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const bool teleported =
    have_pose_ && anchored_ && respawn_jump_m_ > 0.0 && std::hypot(x - veh_x_, y - veh_y_) > respawn_jump_m_;
  if (teleported) {
    resetTrajectory();
    RCLCPP_INFO(get_logger(), "Pose jumped; reset the maneuver");
  }
  veh_x_ = x;
  veh_y_ = y;
  veh_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  have_pose_ = true;
  moved_ = moved_ || (!teleported && veh_speed_ > finish_speed_mps_);
  if (!anchored_) {
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    core_->anchor(x, y, yaw);
    anchored_ = true;
  }
  core_->updateWindow(x, y);
  if (core_->distanceToPath() > max_path_deviation_m_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Vehicle is %.2f m from the maneuver", core_->distanceToPath());
  }
}

void FakePlannerNode::startTrajectoryCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  if (!core_ || !trajectory_pub_ || !trajectory_pub_->is_activated() || !core_->ready() || !have_pose_) {
    response->success = false;
    response->message = "Planner is not active with anchored odometry; wait for ready status";
    return;
  }
  if (finished_) {
    response->success = false;
    response->message = "Maneuver already finished; call reset to run it again";
    return;
  }
  releaseTrajectory();
  publishState("running");
  response->success = true;
  response->message = "Trajectory started";
}

void FakePlannerNode::stopTrajectoryCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  trajectory_started_ = false;
  explicitly_stopped_ = true;
  publishState("stopped");
  response->success = true;
  response->message = "Trajectory stopped; controller will fall back to standby";
}

void FakePlannerNode::resetTrajectoryCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const std::lock_guard<std::mutex> lock(core_mutex_);
  if (!core_) {
    response->success = false;
    response->message = "Planner is not configured";
    return;
  }
  resetTrajectory();
  response->success = true;
  response->message = anchor_to_first_pose_ ? "Reset; re-anchoring on next odom" : "Reset to nearest point";
}

// Session snapshots
void FakePlannerNode::publishState(const std::string & state)
{
  if (status_pub_ && status_pub_->is_activated()) {
    std_msgs::msg::String message;
    message.data = state;
    status_pub_->publish(message);
  }
}

void FakePlannerNode::publishReference()
{
  if (!reference_pub_ || !reference_pub_->is_activated() || !core_->ready()) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header = core_->reference().header;
  path.header.stamp = now();
  for (const auto & point : core_->reference().points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = point.pose;
    path.poses.push_back(pose);
  }
  reference_pub_->publish(path);
}

// Visualization
void FakePlannerNode::publishMarkers(const wato_trajectory_msgs::msg::Trajectory & trajectory)
{
  if (!marker_pub_ || !marker_pub_->is_activated()) {
    return;
  }
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  double limit_speed = 0.0;
  for (const auto & point : trajectory.points) {
    limit_speed = std::max(limit_speed, static_cast<double>(point.max_speed));
  }
  if (limit_speed <= 0.0) {
    limit_speed = 1.0;
  }

  auto header = trajectory.header;
  header.stamp = now();

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "trajectory_velocity";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = point.pose;

    // Size correlates with speed: base 0.1 m, up to 0.5 m at the top speed.
    const double speed_ratio = std::max(0.0, std::min(1.0, point.max_speed / limit_speed));
    const double diameter = 0.1 + (0.4 * speed_ratio);

    // Label every fourth point; marker namespaces keep sphere and label IDs separate.
    if (i % 4 == 0) {
      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "trajectory_speed_labels";
      label.id = static_cast<int>(i);
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

      std::ostringstream speed_text;
      speed_text << std::fixed << std::setprecision(1) << point.max_speed << " m/s";
      label.text = speed_text.str();
      markers.markers.push_back(label);
    }

    sphere.scale.x = diameter;
    sphere.scale.y = diameter;
    sphere.scale.z = diameter;

    sphere.color.r = 0.6f;
    sphere.color.g = 0.2f;
    sphere.color.b = 0.8f;
    sphere.color.a = 0.8f;

    markers.markers.push_back(sphere);
  }

  marker_pub_->publish(markers);
}

}  // namespace fake_planner

RCLCPP_COMPONENTS_REGISTER_NODE(fake_planner::FakePlannerNode)
