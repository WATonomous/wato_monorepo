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
#include <roscco_msg/msg/autonomy_state.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <roscco_msg/msg/steering_torque.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace oscc_interfacing
{

/**
 * @brief ROS 2 node for interfacing with OSCC (Open Source Car Control).
 *
 * Manages the OSCC CAN-based vehicle control system: arming/disarming modules,
 * forwarding steering and throttle commands, and publishing feedback data
 * (wheel speeds, steering angle). Uses callback groups to serialize OSCC API
 * access while allowing feedback publishing to run independently.
 */
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

  /**
   * @brief Lifecycle of the steering actuator with respect to disengagement.
   *
   * ENGAGED:     normal operation, autonomy commands are applied.
   * DISENGAGING: a graceful handover is in progress; steering torque is being
   *              ramped to zero. Autonomy commands are ignored during this phase.
   * DISABLED:    OSCC boards are disabled, the driver has full control.
   */
  enum class DisengageState
  {
    ENGAGED,
    DISENGAGING,
    DISABLED
  };

  /**
   * @brief Snapshot of all four wheel speeds for atomic group read/write.
   */
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

  // Steering torque — single atomic, no tearing possible
  std::atomic<float> latest_steering_torque_{0.0f};
  std::atomic<bool> has_steering_torque_{false};

private:
  /**
   * @brief Declares parameters and initializes the OSCC library, publishers, and timers.
   */
  void configure();

  /**
   * @brief Handles incoming Roscco commands (steering + throttle).
   *
   * Applies steering torque and throttle commands to the vehicle via OSCC API.
   * Only processes commands when the system is armed.
   * @param msg Roscco message containing forward and steering values.
   */
  void roscco_callback(const roscco_msg::msg::Roscco::ConstSharedPtr msg);

  /**
   * @brief Arms or disarms the OSCC modules via service call.
   * @param request Boolean request — true to arm, false to disarm.
   * @param response Success status and message describing the result.
   */
  void arm_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Polls for OSCC override and fault events from the CAN thread.
   *
   * Called on a timer. Checks atomic flags set by CAN callbacks and handles
   * disarming on override or fault.
   */
  void process_events();

  /**
   * @brief Publishes wheel speed and steering angle feedback from CAN data.
   *
   * Runs in its own callback group (Group B) so it does not block OSCC API calls.
   */
  void publish_feedback();

  /**
   * @brief Publishes the current armed state on a timer.
   */
  void is_armed_timer_callback();

  /**
   * @brief Checks an OSCC API result and disarms on error.
   * @param result The OSCC result code to check.
   * @return The same result code passed in.
   */
  oscc_result_t handle_any_errors(oscc_result_t result);

  /**
   * @brief Starts a graceful disengage: ramps steering torque to zero over time.
   *
   * Captures the currently-applied steering torque and the ramp start time, cuts
   * throttle immediately, and transitions to DISENGAGING. The ramp itself is
   * advanced on the event timer via tick_disengage(); this call does not block.
   */
  void begin_disengage();

  /**
   * @brief Advances the steering-torque rampdown by one event-timer tick.
   *
   * Computes the linear ramp fraction from the elapsed time and publishes the
   * scaled steering torque. When the ramp completes, disables the OSCC modules
   * and transitions to DISABLED. No-op unless state is DISENGAGING.
   */
  void tick_disengage();

  /**
   * @brief Disables the configured OSCC modules immediately (no ramp).
   *
   * Honours enable_all_/enable_steering_/enable_throttle_/enable_brakes_.
   * @param disabled_modules Out param describing which modules were disabled.
   * @return true if all configured modules disabled successfully.
   */
  bool disable_modules(std::string & disabled_modules);

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr oscc_api_group_;  // Group A
  rclcpp::CallbackGroup::SharedPtr feedback_group_;  // Group B

  // ROS Interfaces
  rclcpp::Subscription<roscco_msg::msg::Roscco>::SharedPtr roscco_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub_;
  rclcpp::Publisher<roscco_msg::msg::AutonomyState>::SharedPtr autonomy_state_pub_;
  rclcpp::Publisher<roscco_msg::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<roscco_msg::msg::SteeringAngle>::SharedPtr steering_angle_pub_;
  rclcpp::Publisher<roscco_msg::msg::SteeringTorque>::SharedPtr steering_torque_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr is_armed_timer_;
  rclcpp::TimerBase::SharedPtr event_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // Arm state — atomic so the default-group status timer and the destructor can
  // read it safely while Group A callbacks mutate it.
  std::atomic<bool> is_armed_{false};

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

  // Graceful disengage parameters
  bool enable_graceful_disarm_{true};  // If false, manual disarm disables boards instantly
  double disarm_ramp_ms_{600.0};  // Steering torque rampdown duration on graceful disarm

  // Command state — protected by Group A serialization
  float last_forward_{0.0};
  float last_steering_torque_cmd_{0.0};  // Last steering torque actually sent (incl. deadzone)

  // Graceful disengage state — written only in Group A, but atomic so the
  // default-group status timer can publish it without a data race.
  std::atomic<DisengageState> disengage_state_{DisengageState::DISABLED};
  float disengage_initial_torque_{0.0};  // Steering torque at the start of the ramp
  rclcpp::Time disengage_start_time_;  // When the current ramp began
};

}  // namespace oscc_interfacing
