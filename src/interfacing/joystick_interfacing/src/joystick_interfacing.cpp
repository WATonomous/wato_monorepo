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

#include "joystick_interfacing/joystick_interfacing.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>

namespace joystick_node
{

JoystickNode::JoystickNode(const rclcpp::NodeOptions & options)
: Node("joystick_node", options)

{
  configure();
  RCLCPP_INFO(this->get_logger(), "JoystickNode initialized");
}

void JoystickNode::configure()
{
  // -------------------------
  // Declare parameters (no default values)
  // -------------------------
  this->declare_parameter<int>("enable_axis");
  this->declare_parameter<int>("deadman_axis");
  this->declare_parameter<int>("steering_axis");
  this->declare_parameter<int>("throttle_axis");

  this->declare_parameter<double>("max_speed", 2.0);
  this->declare_parameter<double>("max_steering_angle", 0.5);

  this->declare_parameter<bool>("invert_steering", false);
  this->declare_parameter<bool>("invert_throttle", false);
  // -------------------------
  // Read parameters
  // -------------------------

  enable_axis_ = this->get_parameter("enable_axis").as_int();
  deadman_axis_ = this->get_parameter("deadman_axis").as_int();

  steering_axis_ = this->get_parameter("steering_axis").as_int();
  throttle_axis_ = this->get_parameter("throttle_axis").as_int();

  max_speed_ = this->get_parameter("max_speed").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

  invert_steering_ = this->get_parameter("invert_steering").as_bool();
  invert_throttle_ = this->get_parameter("invert_throttle").as_bool();

  // -------------------------
  // Setup pubs/subs
  // -------------------------
  ackermann_drive_stamped_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/joystick/ackermann", rclcpp::QoS(10));
  idle_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/joystick/is_idle", rclcpp::QoS(10));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::QoS(10), std::bind(&JoystickNode::joy_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: enable_axis=%d deadman_axis=%d steering_axis=%d throttle_axis=%d max_speed=%.3f max_steer=%.3f",
    enable_axis_,
    deadman_axis_,
    steering_axis_,
    throttle_axis_,
    max_speed_,
    max_steering_angle_);
}

double JoystickNode::get_axis(const sensor_msgs::msg::Joy & msg, int axis_index) const
{
  if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size()) {
    return 0.0;
  }
  return static_cast<double>(msg.axes[axis_index]);
}

void JoystickNode::publish_neutral_state(bool is_idle)
{
  publish_idle_state(is_idle);
  publish_zero_command();
}

void JoystickNode::publish_idle_state(bool is_idle)
{
  std_msgs::msg::Bool msg;
  msg.data = is_idle;
  idle_state_pub_->publish(msg);
}

void JoystickNode::publish_zero_command()
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = this->now();
  cmd.drive.speed = 0.0;
  cmd.drive.steering_angle = 0.0;
  ackermann_drive_stamped_pub_->publish(cmd);
}

void JoystickNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  // -------------------------
  // Safety gating
  // Two-stage: enable AND deadman must be held
  // -------------------------
  const bool enable_pressed = get_axis(*msg, enable_axis_) <= -0.9;
  const bool deadman_pressed = get_axis(*msg, deadman_axis_) <= -0.9;

  // If enable is not held, fully disarm and stop.
  if (!enable_pressed) {
    deadman_armed_ = false;
    enable_prev_ = false;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 500, "Safety not met (enable_axis=false) -> publishing zero command");
    publish_neutral_state(true);
    return;
  }

  // Enable is held. If it was just pressed, reset arming requirement.
  const bool enable_rising_edge = enable_pressed && !enable_prev_;
  enable_prev_ = enable_pressed;
  if (enable_rising_edge) {
    deadman_armed_ = false;
  }

  // Arm once deadman is pressed while enable is held.
  if (!deadman_armed_ && deadman_pressed) {
    deadman_armed_ = true;
  }

  // Not armed yet -> stop.
  if (!deadman_armed_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "Safety not met (enable held, deadman not yet pressed) -> publishing zero command");
    publish_neutral_state(true);
    return;
  }

  // -------------------------
  // Read + process axes
  // -------------------------
  double steer = get_axis(*msg, steering_axis_);
  double throttle = get_axis(*msg, throttle_axis_);

  if (invert_steering_) {
    steer = -steer;
  }

  if (invert_throttle_) {
    throttle = -throttle;
  }

  // -------------------------
  // Scale to physical commands
  // -------------------------
  const double steering_angle = std::clamp(steer, -1.0, 1.0) * max_steering_angle_;
  const double speed = std::clamp(throttle, -1.0, 1.0) * max_speed_;

  ackermann_msgs::msg::AckermannDriveStamped cmd_stamped;
  cmd_stamped.header.stamp = this->now();
  cmd_stamped.drive.steering_angle = steering_angle;
  cmd_stamped.drive.speed = speed;
  ackermann_drive_stamped_pub_->publish(cmd_stamped);
  publish_idle_state(false);  // Joystick is active, so NOT idle
}

}  // namespace joystick_node

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_node::JoystickNode)
