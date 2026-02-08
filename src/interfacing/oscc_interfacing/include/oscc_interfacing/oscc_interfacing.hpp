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

extern "C"
{
#include <oscc.h>
}

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

/*

This node does these things:

- Subscribes to /roscco

- Publishes to /oscc_interfacing/is_armed (Just a bool, 100HZ)

- Publishes to /oscc_interfacing/wheel_speeds (4 floats, one per wheel)
- Publishes to /oscc_interfacing/steering_angle (float, degrees, 0 = centered)
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
  ~OsccInterfacingNode();

  std::mutex arm_mutex_;
  bool is_armed_{false};

  // Thread-safe data queues for CAN callbacks (public for free function access)
  struct WheelSpeedData { 
    std::atomic<float> ne, nw, se, sw; 
    WheelSpeedData() : ne(0), nw(0), se(0), sw(0) {}
  };
  struct SteeringAngleData { 
    std::atomic<float> angle;
    SteeringAngleData() : angle(0) {}
  };
  enum class OverrideType { BRAKE, THROTTLE, STEERING };
  enum class FaultType { BRAKE_FAULT, STEERING_FAULT, THROTTLE_FAULT };
  
  // Use atomic flags instead of mutex for signal-safe operation
  std::atomic<bool> has_wheel_data_{false};
  std::atomic<bool> has_steering_data_{false};
  std::atomic<bool> has_override_{false};
  std::atomic<bool> has_fault_{false};
  
  // Single data slots (signal handlers write, timer reads)
  WheelSpeedData latest_wheel_data_;
  SteeringAngleData latest_steering_data_;
  OverrideType latest_override_;
  FaultType latest_fault_;

  /**
   * @brief Publishes wheel speeds (4 floats)
   */
  void publish_wheel_speeds(float NE, float NW, float SE, float SW);

  /**
   * @brief Publishes steering wheel angle in degrees
   */
  void publish_steering_angle(float angle_degrees);

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
   * @brief Handles fatal errors from OSCC API calls
   * Attempts to disarm all boards
   */
  oscc_result_t handle_any_errors(oscc_result_t result);

  // ROS Interfaces
  rclcpp::Subscription<roscco_msg::msg::Roscco>::SharedPtr roscco_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub_;
  rclcpp::Publisher<roscco_msg::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<roscco_msg::msg::SteeringAngle>::SharedPtr steering_angle_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  // Timer for 100Hz is_armed publication
  rclcpp::TimerBase::SharedPtr is_armed_timer_;

  // Status tracking
  int is_armed_publish_rate_hz;
  int oscc_can_bus_;
  double steering_scaling_{1.0};
  bool disable_boards_on_fault_{false};
  double steering_conversion_factor_{15.7}; // Steering wheel to wheel angle

  float last_forward_{0.0};
  rclcpp::Time last_message_time_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::TimerBase::SharedPtr data_process_timer_;
  
  void process_queued_data();
};

}  // namespace oscc_interfacing
