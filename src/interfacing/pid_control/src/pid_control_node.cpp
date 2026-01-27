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
  this->declare_parameter<std::string>("ackermann_topic", "/ackermann");
  this->declare_parameter<std::string>("feedback_topic", "/steering_meas");  // TODO(shawn): change to actual topic
  this->declare_parameter<std::string>("roscco_topic", "/roscco");  // TODO(shawn): change to actual topic
  this->declare_parameter<double>("update_rate", 100.0);

  std::string ackermann_topic = this->get_parameter("ackermann_topic").as_string();
  std::string feedback_topic = this->get_parameter("feedback_topic").as_string();
  std::string roscco_topic = this->get_parameter("roscco_topic").as_string();
  double update_rate = this->get_parameter("update_rate").as_double();

  // Initialize PID
  pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "pid_control",
    "/pid_state",
    true);
  pid_ros_->initialize_from_ros_parameters();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    ackermann_topic, rclcpp::QoS(10), std::bind(&PidControlNode::ackermann_callback, this, std::placeholders::_1));

  feedback_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    feedback_topic, rclcpp::QoS(10), std::bind(&PidControlNode::feedback_callback, this, std::placeholders::_1));

  // Publisher
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>(roscco_topic, rclcpp::QoS(10));

  // Timer
  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&PidControlNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "PID Control Node initialized at %.1f Hz", update_rate);
}

void PidControlNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  setpoint_ = msg->drive.steering_angle;
  setpoint_received_ = true;
}

void PidControlNode::feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  feedback_ = msg->data;
  feedback_received_ = true;
}

void PidControlNode::control_loop()
{
  rclcpp::Time now = this->now();
  rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.nanoseconds() <= 0) {
    return;
  }

  if (!setpoint_received_ || !feedback_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for setpoint and feedback signals...");
    return;
  }

  double error = setpoint_ - feedback_;
  double command = pid_ros_->compute_command(error, dt);

  roscco_msg::msg::Roscco msg;
  msg.header.stamp = now;
  msg.steering = static_cast<float>(command);
  msg.forward = 0.0f;

  roscco_pub_->publish(msg);
}

}  // namespace pid_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::PidControlNode)
