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
  this->declare_parameter<int>("oscc_can_bus", 0);

  // Read parameters
  is_armed_ = false;
  is_armed_publish_rate_hz = this->get_parameter("is_armed_publish_rate_hz").as_int();
  oscc_can_bus_ = this->get_parameter("oscc_can_bus").as_int();

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
  
  if (oscc_init() != OSCC_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSCC library");
  } else {
    RCLCPP_INFO(this->get_logger(), "OSCC library initialized successfully");
  }

  if (oscc_open(0) != OSCC_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open OSCC communication");
  } else {
    RCLCPP_INFO(this->get_logger(), "OSCC communication opened successfully");
  }

}

void OsccInterfacingNode::roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg)
{

  // if not armed ignore
  if(!is_armed_) {
    RCLCPP_WARN(this->get_logger(), "Vehicle not armed, Ignoring roscco message");
    return;
  }

  // Check if at least 40ms has passed since last message
  // To not overload CAN (targeting 20HZ, 50-40=10ms leeway)
  auto now = rclcpp::Clock().now();
  if ((now - last_message_time_).nanoseconds() < 40000000) {  // 40ms in nanoseconds
    RCLCPP_WARN(this->get_logger(), "Message too soon, Ignoring roscco message to avoid CAN overload");
    return;
  }
  last_message_time_ = now;

  float brake = 0.0;
  float throttle = 0.0;
  float forward = msg->forward;
  float steering = msg->steering;

  // If forward is positive, set throttle; if negative, set brake
  if(forward >= 0.0) {
    throttle = forward;
    brake = 0.0;
  } else {
    throttle = 0.0;
    brake = -forward;
  }

  /**
   * @brief Check against past values for a smooth transition order
   * between throttle and brake
   */
  if (forward > 0.0 && last_forward_ < 0.0) {
    // Transitioning from brake to throttle 
    // first reduce brake, then apply throttle
    handle_any_errors(oscc_publish_brake_pressure(brake));
    handle_any_errors(oscc_publish_throttle_position(throttle));

  } else {
    // Transitioning from throttle to brake
    // first reduce throttle, then apply brake

    // Also handles base case of no transition
    handle_any_errors(oscc_publish_throttle_position(throttle));
    handle_any_errors(oscc_publish_brake_pressure(brake));
  }

  last_forward_ = forward;
}

void OsccInterfacingNode::arm_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) { // data is the boolean, true = arm, false = disarm
    // Arm the vehicle
    if (oscc_enable() == OSCC_OK) {
      is_armed_ = true;
      response->success = true;
      response->message = "Vehicle armed successfully";
      RCLCPP_INFO(get_logger(), "Vehicle armed");
    } else {
      response->success = false;
      response->message = "Failed to arm vehicle";
      RCLCPP_ERROR(get_logger(), "Failed to arm vehicle");
    }
  } else {
    // Disarm the vehicle
    if (oscc_disable() == OSCC_OK) {
      is_armed_ = false;
      response->success = true;
      response->message = "Vehicle disarmed successfully";
      RCLCPP_INFO(get_logger(), "Vehicle disarmed");
    } else {
      response->success = false;
      response->message = "Failed to disarm vehicle";
      RCLCPP_FATAL(get_logger(), "!!!!!! Failed to disarm vehicle");
    }
  }
}

void OsccInterfacingNode::is_armed_timer_callback()
{
  std_msgs::msg::Bool msg;
  msg.data = is_armed_;
  is_armed_pub_->publish(msg);
}

void OsccInterfacingNode::publish_wheel_speeds(const std::vector<float> & speeds)
{
  // TODO: Create and publish wheel speeds message
}

void OsccInterfacingNode::publish_steering_wheel_angle(float angle_degrees)
{
  // TODO: Create and publish steering wheel angle message
}

oscc_result_t OsccInterfacingNode::handle_any_errors(oscc_result_t result)
{
  if(result == OSCC_OK) {
    return OSCC_OK;
  }
  is_armed_ = false;
  RCLCPP_ERROR(this->get_logger(), "Error from OSCC API: %d, ATTEMPTING TO DISARM ALL BOARDS", result);
  // Attempt to disarm all boards
  if(oscc_disable() != OSCC_OK) {
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
    RCLCPP_FATAL(this->get_logger(), "!! FAILED TO DISARM ALL BOARDS, MANUAL INTERVENTION REQUIRED !!");
  } else {
    RCLCPP_INFO(this->get_logger(), "All boards disarmed successfully after error");
  }
  return result; // pass error up
}

}  // namespace oscc_interfacing

RCLCPP_COMPONENTS_REGISTER_NODE(oscc_interfacing::OsccInterfacingNode)