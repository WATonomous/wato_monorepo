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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace joystick_node
{

/**
 * @brief Converts joystick input to Ackermann drive commands.
 *
 * Subscribes to raw joystick data and publishes AckermannDriveStamped
 * commands. Includes safety gating via enable axis and idle state tracking.
 */
class JoystickNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  enum class JoystickState : int8_t
  {
    NULL_STATE = 0,
    ACKERMANN = 1,
    ROSSCO = 2
  };

  explicit JoystickNode(const rclcpp::NodeOptions & options);

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configures the node, including parameters and internal state.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates the node, enabling publishers and timers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates the node, disabling publishers and timers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleans up the node, releasing resources.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shuts down the node.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Local smoothing state for the deadman (enable-axis) press/release
   * transition, applied to steering torque only — independent of the OSCC
   * arm/disarm ramp in oscc_interfacing. Mirrors that node's DisengageState.
   *
   * ENGAGED:     deadman held past the ramp window; steering authority is 1.0.
   * ENGAGING:    deadman freshly pressed; steering authority ramping 0 -> 1.
   * DISENGAGING: deadman freshly released; steering authority ramping toward 0.
   * DISABLED:    deadman released and the ramp-down has completed; steering
   *              authority is 0.0 and idle/mask topics report idle.
   */
  enum class DeadmanState : int8_t
  {
    ENGAGED = 0,
    ENGAGING = 1,
    DISENGAGING = 2,
    DISABLED = 3
  };

  /**
   * @brief Main hot-loop: process joystick, publish ackermann
   */
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /**
   * @brief Safe accessors (avoid out-of-range)
   */
  double get_axis(const sensor_msgs::msg::Joy & msg, int axis_index) const;
  bool get_button(const sensor_msgs::msg::Joy & msg, int button_index) const;

  /**
   * @brief Starts a steering-authority ramp-up from the current authority
   * (handles rapid re-press mid ramp-down instead of double-jerking through zero).
   */
  void begin_engage();

  /**
   * @brief Starts a steering-authority ramp-down from the current authority.
   */
  void begin_disengage();

  /**
   * @brief Advances an in-progress ramp-up; transitions ENGAGING -> ENGAGED on completion.
   */
  void tick_engage();

  /**
   * @brief Advances an in-progress ramp-down; transitions DISENGAGING -> DISABLED on completion.
   */
  void tick_disengage();

  /**
   * @brief Current steering authority scale in [0, 1]. 1.0 when ENGAGED, 0.0 when
   * DISABLED, linearly ramping during ENGAGING/DISENGAGING.
   */
  float engage_authority_scale() const;

  /**
   * @brief True once the deadman ramp has fully settled at zero (DISABLED).
   * Drives the idle/mask topics independently of the raw enable-axis reading.
   */
  bool is_deadman_idle() const;

  /**
   * @brief Scales steering by engage_authority_scale() (ramped) and throttle by
   * whether the deadman is currently held (instant, no ramp — throttle must cut
   * immediately for safety), then publishes to whichever output topic is selected.
   * @param steer_raw Live steering axis value, pre-clamp, post-invert.
   * @param throttle_raw Live throttle axis value, pre-clamp, post-invert.
   * @param enable_held Whether the deadman axis is currently held.
   */
  void publish_scaled_command(double steer_raw, double throttle_raw, bool enable_held);

  /**
   * @brief Periodic tick, independent of joy message arrival, that animates the
   * steering ramp between joystick messages. No-op outside ENGAGING/DISENGAGING.
   */
  void ramp_timer_callback();

  /**
   * @brief Publish is idle state if no new input to joystick for a certain time
   */
  void publish_ackermann_idle_state(bool is_idle);

  /**
   * @brief Publish idle state for the ROSCCO mux (true = joystick not providing ROSCCO commands).
   */
  void publish_roscco_idle_state(bool is_idle);

  /**
   * @brief Publishes the current joystick state.
   */
  void publish_state(JoystickState state);

  /**
   * @brief Publishes a zero-velocity AckermannDriveStamped command (coasting).
   */
  void publish_zero_command();

  /**
   * @brief Triggers a vibration sequence.
   * @param count Number of vibration pulses.
   * @param duration_ms Duration of each pulse in ms.
   */
  void vibrate(int count, int duration_ms);

  /**
   * @brief Handles the vibration sequence pulses.
   */
  void vibration_timer_callback();

  /**
   * @brief Updates local is_armed state.
   */
  void is_armed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_stamped_pub_;
  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr roscco_joystick_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr idle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr roscco_idle_state_pub_;
  // state status
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_pub_;

  // Arming interface
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;

  // Vibration timer
  rclcpp::TimerBase::SharedPtr vibration_timer_;

  // Deadman steering ramp timer — animates the ramp independent of joy message arrival
  rclcpp::TimerBase::SharedPtr ramp_timer_;

  int enable_axis_;  // index of enable axis (shoulder button)
  int toggle_button_;  // index of toggle button
  int arming_button_;  // index of arming button

  int steering_axis_;  // index of steering axis (left/right joystick)
  int throttle_axis_;  // index of throttle axis (left/right joystick)

  double ackermann_max_speed_;  // maximum speed
  double ackermann_max_steering_angle_;  // maximum steering angle

  double roscco_max_speed_;  // maximum speed
  double roscco_max_steering_angle_;  // maximum steering angle

  bool invert_steering_;  // invert steering direction
  bool invert_throttle_;  // invert throttle direction

  bool use_roscco_topic_{false};  // toggle between /joystick/ackermann and /joystick/roscco
  bool prev_toggle_button_pressed_{false};  // previous state of toggle button for edge detection

  // Arming state
  bool is_armed_{false};
  bool prev_arming_button_pressed_{false};

  // Deadman steering ramp state (steering only — throttle is gated instantly, no ramp)
  DeadmanState deadman_state_{DeadmanState::DISABLED};
  float steering_authority_{0.0f};
  float engage_initial_scale_{0.0f};
  float disengage_initial_scale_{0.0f};
  rclcpp::Time engage_start_time_;
  rclcpp::Time disengage_start_time_;
  bool prev_enable_pressed_{false};
  double last_steer_raw_{0.0};
  double last_throttle_raw_{0.0};

  // Deadman steering ramp parameters
  bool enable_deadman_ramp_{true};
  double deadman_engage_ramp_ms_{600.0};
  double deadman_disengage_ramp_ms_{600.0};
  double ramp_timer_hz_{50.0};

  // Vibration parameters
  double toggle_vibration_intensity_;
  int toggle_vibration_duration_ms_;

  // Current vibration settings
  int current_vibration_duration_ms_{100};

  // Vibration sequence state
  int vibration_pulses_remaining_{0};
  bool vibration_on_{false};
};
}  // namespace joystick_node
