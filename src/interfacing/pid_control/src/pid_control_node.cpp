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

#include "pid_control/pid_control_node.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace pid_control
{

PidControlNode::PidControlNode(const rclcpp::NodeOptions & options)
: Node("pid_control_node", options)
{
  // Declare topics
  this->declare_parameter<double>("update_rate", 100.0);

  double update_rate = this->get_parameter("update_rate").as_double();

  // Initialize Steering PID
  steering_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "steering_pid",
    "/steering_pid_state",
    true);
  steering_pid_ros_->initialize_from_ros_parameters();

  // Initialize Velocity PID
  velocity_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "velocity_pid",
    "/velocity_pid_state",
    true);
  velocity_pid_ros_->initialize_from_ros_parameters();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann", rclcpp::QoS(10), std::bind(&PidControlNode::ackermann_callback, this, std::placeholders::_1));

  steering_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "steering_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlNode::steering_feedback_callback, this, std::placeholders::_1));

  velocity_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "velocity_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlNode::velocity_feedback_callback, this, std::placeholders::_1));

  // Publisher
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(10));

  // Timer
  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&PidControlNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "PID Control Node initialized at %.1f Hz", update_rate);
}

void PidControlNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  steering_setpoint_ = msg->drive.steering_angle;
  velocity_setpoint_ = msg->drive.speed;
  ackermann_received_ = true;
}

void PidControlNode::steering_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  steering_meas_ = msg->data;
  steering_meas_received_ = true;
}

void PidControlNode::velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  velocity_meas_ = msg->data;
  velocity_meas_received_ = true;
}

void PidControlNode::control_loop()
{
  rclcpp::Time now = this->now();
  rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.nanoseconds() <= 0) {
    return;
  }

  if (!ackermann_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for ackermann setpoint...");
    return;
  }

  if (!steering_meas_received_ || !velocity_meas_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Waiting for feedback signals (steering: %s, velocity: %s)...",
      steering_meas_received_ ? "OK" : "MISSING",
      velocity_meas_received_ ? "OK" : "MISSING");
    return;
  }

  // Compute Steering Command (Torque)
  double steering_error = steering_setpoint_ - steering_meas_;
  double steering_command = steering_pid_ros_->compute_command(steering_error, dt);

  // Compute Velocity Command
  double velocity_error = velocity_setpoint_ - velocity_meas_;
  double velocity_command = velocity_pid_ros_->compute_command(velocity_error, dt);

  // Publish Roscco message
  roscco_msg::msg::Roscco msg;
  msg.header.stamp = now;
  msg.steering = static_cast<float>(steering_command);
  msg.forward = static_cast<float>(velocity_command);

  roscco_pub_->publish(msg);
}

}  // namespace pid_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::PidControlNode)
