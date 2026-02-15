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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

/*

Threading model:

  CAN thread (OSCC library)    — writes sensor data and events via atomics/mutex
  Group A (MutuallyExclusive)  — serializes all OSCC API calls:
                                   roscco_callback, arm_service_callback, process_events
  Group B (MutuallyExclusive)  — feedback publishing (independent of OSCC API):
                                   publish_feedback
  Default group                — is_armed_timer_callback (trivial, runs freely)

*/

namespace oscc_interfacing
{

class OsccInterfacingNode : public rclcpp::Node
{
public:
  explicit OsccInterfacingNode(const rclcpp::NodeOptions & options);
  ~OsccInterfacingNode();

  // --- CAN thread → ROS thread data transfer (public for free function callbacks) ---

  enum class OverrideType
  {
    BRAKE,
    THROTTLE,
    STEERING
  };
  enum class FaultType
  {
    BRAKE_FAULT,
    STEERING_FAULT,
    THROTTLE_FAULT
  };

  // Wheel speed snapshot — protected by wheel_data_mutex_ for atomic group read/write
  struct WheelSpeedSnapshot
  {
    float ne{0}, nw{0}, se{0}, sw{0};
  };

  std::mutex wheel_data_mutex_;
  WheelSpeedSnapshot latest_wheel_data_;
  std::atomic<bool> has_wheel_data_{false};

  // Steering angle — single atomic, no tearing possible
  std::atomic<float> latest_steering_angle_{0.0f};
  std::atomic<bool> has_steering_data_{false};

  // Override and fault events — atomic enums to avoid UB
  std::atomic<OverrideType> latest_override_{OverrideType::BRAKE};
  std::atomic<FaultType> latest_fault_{FaultType::BRAKE_FAULT};
  std::atomic<bool> has_override_{false};
  std::atomic<bool> has_fault_{false};

private:
  void configure();

  // Group A callbacks (MutuallyExclusive — serializes OSCC API access)
  void roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg);
  void arm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void process_events();

  // Group B callback (MutuallyExclusive — feedback publishing, independent of OSCC API)
  void publish_feedback();

  // Default group
  void is_armed_timer_callback();

  oscc_result_t handle_any_errors(oscc_result_t result);

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr oscc_api_group_;   // Group A
  rclcpp::CallbackGroup::SharedPtr feedback_group_;    // Group B

  // ROS Interfaces
  rclcpp::Subscription<roscco_msg::msg::Roscco>::SharedPtr roscco_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub_;
  rclcpp::Publisher<roscco_msg::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<roscco_msg::msg::SteeringAngle>::SharedPtr steering_angle_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr is_armed_timer_;
  rclcpp::TimerBase::SharedPtr event_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // Arm state — protected by arm_mutex_
  std::mutex arm_mutex_;
  bool is_armed_{false};

  // Parameters
  int is_armed_publish_rate_hz;
  int oscc_can_bus_;
  double steering_scaling_{1.0};
  bool disable_boards_on_fault_{false};
  double steering_conversion_factor_{15.7};
  double steering_torque_deadzone_pos_{0.0};
  double steering_torque_deadzone_neg_{0.0};

  bool enable_all_{true};
  bool enable_steering_{true};
  bool enable_throttle_{true};
  bool enable_brakes_{true};

  // Command state — protected by Group A serialization
  float last_forward_{0.0};
};

}  // namespace oscc_interfacing
