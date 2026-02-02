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
class JoystickNode : public rclcpp::Node
{
public:
  enum class JoystickState : int8_t
  {
    NULL_STATE = 0,
    ACKERMANN = 1,
    ROSSCO = 2
  };

  explicit JoystickNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Loads parameters, initializes pubs/subs
   */
  void configure();

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
   * @brief Publishes both the idle state and a zero-velocity command.
   */
  void publish_neutral_state(bool is_idle);

  /**
   * @brief Publish is idle state if no new input to joystick for a certain time
   */
  void publish_idle_state(bool is_idle);

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
  // state status
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_pub_;

  // Arming interface
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;

  // Vibration timer
  rclcpp::TimerBase::SharedPtr vibration_timer_;

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
