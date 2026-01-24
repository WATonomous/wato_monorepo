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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <oscc.h>

/*

This node does these things:

- Subscribes to /joystick/roscco

- Publishes to /oscc_interfacing/is_armed (Just a bool, 100HZ)

- Publishes to /oscc_interfacing/wheel_speeds (4 floats, one per wheel)
- Publishes to /oscc_interfacing/steering_wheel_angle (float, degrees, 0 = centered)
      - You can model this with ackermann reference frames and get an odom for
      - speed and angular velocity for localization

- Is a server for the service /oscc_interfacing/arm
      - Attempt to either arm or disarm.
      - SetBool service: true = attempt to arm, false = attempt to disarm
      - Returns success/fail (message empty)

*/

namespace oscc_interfacing
{

class OsccInterfacingNode : public rclcpp::Node
{
public:
  explicit OsccInterfacingNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Loads parameters, initializes pubs/subs/services
   */
  void configure();

  /**
   * @brief Callback for joystick input from /joystick/roscco
   */
  void roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg);

  /**
   * @brief Service callback to arm/disarm the vehicle boards
   * @param request.data true = arm, false = disarm
   */
  void arm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Timer callback to publish is_armed status at 100Hz
   */
  void is_armed_timer_callback();

  /**
   * @brief Publishes wheel speeds (4 floats)
   */
  void publish_wheel_speeds(const std::vector<float> & speeds);

  /**
   * @brief Publishes steering wheel angle in degrees
   */
  void publish_steering_wheel_angle(float angle_degrees);

  /**
   * @brief Handles fatal errors from OSCC API calls
   * Attempts to disarm all boards
   */
  void handle_any_errors(oscc_result_t result);


  // ROS Interfaces
  rclcpp::Subscription<roscco_msg::msg::Roscco>::SharedPtr roscco_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub_;
  rclcpp::Publisher<roscco_msg::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_wheel_angle_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  // Timer for 100Hz is_armed publication
  rclcpp::TimerBase::SharedPtr is_armed_timer_;

  // Status tracking
  bool is_armed_{false};
  int is_armed_publish_rate_hz;
  int oscc_can_bus_;

  float last_forward_{0.0};
  float last_message_time_{0.0};

};

}  // namespace oscc_interfacing
