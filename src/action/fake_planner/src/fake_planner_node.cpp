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

#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace
{

// A CSV line carries no waypoint if it's blank, a '#' comment, or a header row. Data rows
// start with a number, so treat any line whose first non-space character isn't numeric as
// skippable (this also transparently skips the "x,y,speed" header).
bool isSkippableLine(const std::string & line)
{
  for (const char ch : line) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }
    if (ch == '#') {
      return true;
    }
    return !(std::isdigit(static_cast<unsigned char>(ch)) || ch == '+' || ch == '-' || ch == '.');
  }
  return true;  // blank / whitespace-only
}

}  // namespace

namespace fake_planner
{

FakePlannerNode::FakePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fake_planner_node", options)
{
  // Topics / frame
  declare_parameter("trajectory_topic", "trajectory");
  declare_parameter("behaviour_topic", "execute_behaviour");
  declare_parameter("odom_topic", "odom");
  declare_parameter("frame_id", "odom");

  // Behaviour
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("publish_behaviour", true);
  declare_parameter("behaviour", "lane_follow");
  declare_parameter("anchor_to_first_pose", true);

  // Maneuver CSV (x,y,speed rows, authored in frame_id relative to the anchor pose). One
  // maneuver per file, selected from the package's maneuvers/ folder via the launch.
  declare_parameter("maneuver_file", std::string(""));
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  behaviour_topic_ = get_parameter("behaviour_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  publish_behaviour_ = get_parameter("publish_behaviour").as_bool();
  behaviour_ = get_parameter("behaviour").as_string();
  anchor_to_first_pose_ = get_parameter("anchor_to_first_pose").as_bool();
  maneuver_file_ = get_parameter("maneuver_file").as_string();

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "publish_rate_hz must be > 0 (got %.3f)", publish_rate_hz_);
    return CallbackReturn::FAILURE;
  }
  if (maneuver_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "maneuver_file is empty; point it at a maneuver CSV (x,y,speed rows).");
    return CallbackReturn::FAILURE;
  }
  std::string load_error;
  if (!loadManeuver(maneuver_file_, load_error)) {
    RCLCPP_ERROR(get_logger(), "Failed to load maneuver '%s': %s", maneuver_file_.c_str(), load_error.c_str());
    return CallbackReturn::FAILURE;
  }
  // loadManeuver appends x/y/speed together, so the arrays are equal-length by construction.
  if (wp_x_.size() < 2) {
    RCLCPP_ERROR(
      get_logger(), "Maneuver '%s' has %zu waypoint(s); need >= 2.", maneuver_file_.c_str(), wp_x_.size());
    return CallbackReturn::FAILURE;
  }

  trajectory_pub_ = create_publisher<wato_trajectory_msgs::msg::Trajectory>(trajectory_topic_, rclcpp::QoS(10));
  if (publish_behaviour_) {
    behaviour_pub_ = create_publisher<behaviour_msgs::msg::ExecuteBehaviour>(behaviour_topic_, rclcpp::QoS(10));
  }

  // Anchoring lays the waypoints out from the vehicle's pose at launch, so the same file
  // works regardless of where odom's origin is (sim spawn vs. accumulated odom on the car).
  // Without it, waypoints are published verbatim in frame_id.
  if (anchor_to_first_pose_) {
    anchored_ = false;
    trajectory_ready_ = false;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10), std::bind(&FakePlannerNode::odomCallback, this, std::placeholders::_1));
  } else {
    anchor_x_ = 0.0;
    anchor_y_ = 0.0;
    anchor_yaw_ = 0.0;
    anchored_ = true;
    buildTrajectory();
  }

  RCLCPP_INFO(
    get_logger(),
    "Configured: %zu waypoints from '%s', frame '%s', %.1f Hz, anchor=%s, behaviour=%s",
    wp_x_.size(),
    maneuver_file_.c_str(),
    frame_id_.c_str(),
    publish_rate_hz_,
    anchor_to_first_pose_ ? "true" : "false",
    publish_behaviour_ ? behaviour_.c_str() : "(disabled)");
  return CallbackReturn::SUCCESS;
}

bool FakePlannerNode::loadManeuver(const std::string & path, std::string & error)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    error = "cannot open file";
    return false;
  }

  wp_x_.clear();
  wp_y_.clear();
  wp_speed_.clear();

  std::string line;
  std::size_t line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    if (isSkippableLine(line)) {
      continue;
    }

    std::stringstream ss(line);
    std::string x_str;
    std::string y_str;
    std::string speed_str;
    if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',') || !std::getline(ss, speed_str, ',')) {
      error = "line " + std::to_string(line_no) + ": expected 'x,y,speed'";
      return false;
    }

    try {
      wp_x_.push_back(std::stod(x_str));
      wp_y_.push_back(std::stod(y_str));
      wp_speed_.push_back(std::stod(speed_str));
    } catch (const std::exception &) {
      error = "line " + std::to_string(line_no) + ": could not parse numbers from '" + line + "'";
      return false;
    }
  }

  return true;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  trajectory_pub_->on_activate();
  if (behaviour_pub_) {
    behaviour_pub_->on_activate();
  }

  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, std::bind(&FakePlannerNode::timerCallback, this));

  RCLCPP_INFO(get_logger(), "Activated: publishing trajectory at %.1f Hz", publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_->on_deactivate();
  if (behaviour_pub_) {
    behaviour_pub_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_.reset();
  behaviour_pub_.reset();
  odom_sub_.reset();
  trajectory_ready_ = false;
  anchored_ = false;
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

FakePlannerNode::CallbackReturn FakePlannerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  trajectory_pub_.reset();
  behaviour_pub_.reset();
  odom_sub_.reset();
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void FakePlannerNode::buildTrajectory()
{
  const double c = std::cos(anchor_yaw_);
  const double s = std::sin(anchor_yaw_);

  // Apply the SE(2) anchor transform to every waypoint up front so the timer callback only
  // has to restamp and publish.
  std::vector<double> xs(wp_x_.size());
  std::vector<double> ys(wp_x_.size());
  for (std::size_t i = 0; i < wp_x_.size(); ++i) {
    xs[i] = anchor_x_ + wp_x_[i] * c - wp_y_[i] * s;
    ys[i] = anchor_y_ + wp_x_[i] * s + wp_y_[i] * c;
  }

  wato_trajectory_msgs::msg::Trajectory traj;
  traj.header.frame_id = frame_id_;
  traj.points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = xs[i];
    pt.pose.position.y = ys[i];
    pt.pose.position.z = 0.0;

    // Yaw from consecutive points (last point reuses the previous heading).
    double yaw = 0.0;
    if (i + 1 < xs.size()) {
      yaw = std::atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i]);
    } else if (i > 0) {
      yaw = std::atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]);
    }
    pt.pose.orientation.z = std::sin(yaw / 2.0);
    pt.pose.orientation.w = std::cos(yaw / 2.0);

    pt.max_speed = wp_speed_[i];
    traj.points.push_back(pt);
  }

  trajectory_ = traj;
  trajectory_ready_ = true;
}

void FakePlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if (anchored_) {
    return;
  }
  anchor_x_ = msg->pose.pose.position.x;
  anchor_y_ = msg->pose.pose.position.y;
  const auto & q = msg->pose.pose.orientation;
  // Yaw from quaternion (z-up): atan2(2(wz + xy), 1 - 2(yy + zz)).
  anchor_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  anchored_ = true;
  buildTrajectory();

  RCLCPP_INFO(
    get_logger(),
    "Anchored trajectory to pose (%.2f, %.2f, yaw=%.3f) in frame '%s'",
    anchor_x_,
    anchor_y_,
    anchor_yaw_,
    frame_id_.c_str());
}

void FakePlannerNode::timerCallback()
{
  if (!trajectory_ready_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for first odom on '%s' to anchor trajectory...", odom_topic_.c_str());
    return;
  }

  trajectory_.header.stamp = now();
  trajectory_pub_->publish(trajectory_);

  if (behaviour_pub_) {
    behaviour_msgs::msg::ExecuteBehaviour beh;
    beh.behaviour = behaviour_;
    behaviour_pub_->publish(beh);
  }
}

}  // namespace fake_planner

RCLCPP_COMPONENTS_REGISTER_NODE(fake_planner::FakePlannerNode)
