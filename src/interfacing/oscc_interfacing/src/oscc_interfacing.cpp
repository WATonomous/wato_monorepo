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

#include "oscc_interfacing/oscc_interfacing.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>
namespace oscc_interfacing
{

OsccInterfacingNode::OsccInterfacingNode(const rclcpp::NodeOptions & options)
: Node("oscc_interfacing_node", options)
{
  configure();
  RCLCPP_INFO(this->get_logger(), "OsccInterfacingNode initialized");
}

void OsccInterfacingNode::configure()
{
  // Declare parameters
  this->declare_parameter<int>("is_armed_publish_rate_hz", 100);

  // Read parameters
  is_armed_ = false;
  is_armed_publish_rate_hz = this->get_parameter("is_armed_publish_rate_hz").as_int();

  // Create subscription to /joystick/roscco
  roscco_sub_ = this->create_subscription<roscco_msg::msg::Roscco>(
    "/joystick/roscco",
    rclcpp::QoS(1),
    std::bind(&OsccInterfacingNode::roscco_callback, this, std::placeholders::_1));

  // Create publishers
  is_armed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/oscc_interfacing/is_armed",
    rclcpp::QoS(1));

  wheel_speeds_pub_ = this->create_publisher<roscco_msg::msg::WheelSpeeds>(
    "/oscc_interfacing/wheel_speeds",
    rclcpp::QoS(1));

  steering_wheel_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/oscc_interfacing/steering_wheel_angle",
    rclcpp::QoS(1));

  // Create arm service
  arm_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/oscc_interfacing/arm",
    std::bind(&OsccInterfacingNode::arm_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Create 100Hz timer for is_armed publication
  std::chrono::milliseconds interval(1000 / is_armed_publish_rate_hz);
  is_armed_timer_ = this->create_wall_timer(
    interval,
    std::bind(&OsccInterfacingNode::is_armed_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "OsccInterfacingNode configured: armed=, is_armed_publish_rate=%d Hz",
    is_armed_,
    is_armed_publish_rate_hz);
}

void OsccInterfacingNode::roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg)
{
  // TODO: Process joystick input and publish wheel speeds/steering angle
}

void OsccInterfacingNode::arm_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // TODO: Handle arm/disarm request (request->data = true/false)
  // TODO: Set is_armed_ accordingly
  // TODO: Fill response with success and message
}

void OsccInterfacingNode::is_armed_timer_callback()
{
  // TODO: Publish is_armed_ status
}

void OsccInterfacingNode::publish_wheel_speeds(const std::vector<float> & speeds)
{
  // TODO: Create and publish wheel speeds message
}

void OsccInterfacingNode::publish_steering_wheel_angle(float angle_degrees)
{
  // TODO: Create and publish steering wheel angle message
}

}  // namespace oscc_interfacing

RCLCPP_COMPONENTS_REGISTER_NODE(oscc_interfacing::OsccInterfacingNode)