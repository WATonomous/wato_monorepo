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

#include "joystick_interfacing/joystick_interfacing.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp_components/register_node_macro.hpp>

namespace joystick_node
{

JoystickNode::JoystickNode(const rclcpp::NodeOptions & options)
: LifecycleNode("joystick_node", options)
{
  // Declare parameters only - do not read or create pub/sub yet
  this->declare_parameter<int>("enable_axis");
  this->declare_parameter<int>("toggle_button", 0);
  this->declare_parameter<int>("steering_axis");
  this->declare_parameter<int>("throttle_axis");
  this->declare_parameter<int>("arming_button", 0);

  this->declare_parameter<double>("ackermann_max_speed", 2.0);
  this->declare_parameter<double>("ackermann_max_steering_angle", 0.5);

  this->declare_parameter<double>("roscco_max_speed", 1.0);
  this->declare_parameter<double>("roscco_max_steering_angle", 0.3);

  this->declare_parameter<bool>("invert_steering", false);
  this->declare_parameter<bool>("invert_throttle", false);

  this->declare_parameter<double>("toggle_vibration_intensity", 0.5);
  this->declare_parameter<int>("toggle_vibration_duration_ms", 100);

  RCLCPP_INFO(this->get_logger(), "JoystickNode created (unconfigured)");
}

JoystickNode::CallbackReturn JoystickNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Read parameters
  enable_axis_ = this->get_parameter("enable_axis").as_int();
  toggle_button_ = this->get_parameter("toggle_button").as_int();
  arming_button_ = this->get_parameter("arming_button").as_int();
  steering_axis_ = this->get_parameter("steering_axis").as_int();
  throttle_axis_ = this->get_parameter("throttle_axis").as_int();

  ackermann_max_speed_ = this->get_parameter("ackermann_max_speed").as_double();
  ackermann_max_steering_angle_ = this->get_parameter("ackermann_max_steering_angle").as_double();

  roscco_max_speed_ = this->get_parameter("roscco_max_speed").as_double();
  roscco_max_steering_angle_ = this->get_parameter("roscco_max_steering_angle").as_double();

  invert_steering_ = this->get_parameter("invert_steering").as_bool();
  invert_throttle_ = this->get_parameter("invert_throttle").as_bool();

  toggle_vibration_intensity_ = this->get_parameter("toggle_vibration_intensity").as_double();
  toggle_vibration_duration_ms_ = this->get_parameter("toggle_vibration_duration_ms").as_int();

  // Setup publishers
  ackermann_drive_stamped_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/joystick/ackermann", rclcpp::QoS(10));
  roscco_joystick_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("/roscco", rclcpp::QoS(10));
  idle_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/joystick/is_idle", rclcpp::QoS(10));
  state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/joystick/state", rclcpp::QoS(10));
  joy_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", rclcpp::QoS(10));

  // Setup subscribers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::QoS(10), std::bind(&JoystickNode::joy_callback, this, std::placeholders::_1));

  // Arming interface
  arm_client_ = this->create_client<std_srvs::srv::SetBool>("/oscc_interfacing/arm");
  is_armed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/oscc_interfacing/is_armed",
    rclcpp::QoS(10),
    std::bind(&JoystickNode::is_armed_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: enable_axis=%d steering_axis=%d throttle_axis=%d ackermann_max_speed=%.3f ackermann_max_steer=%.3f "
    "roscco_max_speed=%.3f roscco_max_steer=%.3f ",
    enable_axis_,
    steering_axis_,
    throttle_axis_,
    ackermann_max_speed_,
    ackermann_max_steering_angle_,
    roscco_max_speed_,
    roscco_max_steering_angle_);

  return CallbackReturn::SUCCESS;
}

JoystickNode::CallbackReturn JoystickNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

JoystickNode::CallbackReturn JoystickNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivated");

  // Cancel vibration timer if running
  if (vibration_timer_) {
    vibration_timer_->cancel();
    vibration_timer_.reset();
  }

  return CallbackReturn::SUCCESS;
}

JoystickNode::CallbackReturn JoystickNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  // Reset all resources
  joy_sub_.reset();
  ackermann_drive_stamped_pub_.reset();
  roscco_joystick_pub_.reset();
  idle_state_pub_.reset();
  state_pub_.reset();
  joy_feedback_pub_.reset();
  is_armed_sub_.reset();
  arm_client_.reset();
  vibration_timer_.reset();

  // Reset state
  use_roscco_topic_ = false;
  prev_toggle_button_pressed_ = false;
  is_armed_ = false;
  prev_arming_button_pressed_ = false;
  vibration_pulses_remaining_ = 0;
  vibration_on_ = false;

  return CallbackReturn::SUCCESS;
}

JoystickNode::CallbackReturn JoystickNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  // Same cleanup as on_cleanup
  if (vibration_timer_) {
    vibration_timer_->cancel();
    vibration_timer_.reset();
  }

  joy_sub_.reset();
  ackermann_drive_stamped_pub_.reset();
  roscco_joystick_pub_.reset();
  idle_state_pub_.reset();
  state_pub_.reset();
  joy_feedback_pub_.reset();
  is_armed_sub_.reset();
  arm_client_.reset();

  return CallbackReturn::SUCCESS;
}

double JoystickNode::get_axis(const sensor_msgs::msg::Joy & msg, int axis_index) const
{
  if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size()) {
    return 0.0;
  }
  return static_cast<double>(msg.axes[axis_index]);
}

bool JoystickNode::get_button(const sensor_msgs::msg::Joy & msg, int button_index) const
{
  if (button_index < 0 || static_cast<size_t>(button_index) >= msg.buttons.size()) {
    return false;
  }
  return msg.buttons[button_index] != 0;
}

void JoystickNode::publish_neutral_state(bool is_idle)
{
  publish_idle_state(is_idle);
  publish_state(JoystickState::NULL_STATE);
  publish_zero_command();
}

void JoystickNode::publish_idle_state(bool is_idle)
{
  std_msgs::msg::Bool msg;
  msg.data = is_idle;
  idle_state_pub_->publish(msg);
}

void JoystickNode::publish_state(JoystickState state)
{
  std_msgs::msg::Int8 msg;
  msg.data = static_cast<int8_t>(state);
  state_pub_->publish(msg);
}

void JoystickNode::publish_zero_command()
{
  if (use_roscco_topic_) {
    roscco_msg::msg::Roscco cmd;
    cmd.header.stamp = this->now();
    cmd.forward = 0.0;
    cmd.steering = 0.0;
    roscco_joystick_pub_->publish(cmd);
  } else {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    ackermann_drive_stamped_pub_->publish(cmd);
  }
}

void JoystickNode::is_armed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  is_armed_ = msg->data;
}

void JoystickNode::vibrate(int count, int duration_ms)
{
  if (count <= 0) {
    return;
  }
  if (vibration_timer_) {
    vibration_timer_->cancel();
  }
  current_vibration_duration_ms_ = duration_ms;
  vibration_pulses_remaining_ = count;
  vibration_on_ = false;  // Start by turning it ON in the callback
  vibration_timer_callback();
}

void JoystickNode::vibration_timer_callback()
{
  if (vibration_timer_) {
    vibration_timer_->cancel();
  }

  sensor_msgs::msg::JoyFeedback fb;
  fb.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  fb.id = 0;

  if (!vibration_on_) {
    // Turn ON
    vibration_on_ = true;
    fb.intensity = static_cast<float>(toggle_vibration_intensity_);
    joy_feedback_pub_->publish(fb);

    vibration_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(current_vibration_duration_ms_),
      std::bind(&JoystickNode::vibration_timer_callback, this));
  } else {
    // Turn OFF
    vibration_on_ = false;
    fb.intensity = 0.0f;
    joy_feedback_pub_->publish(fb);

    vibration_pulses_remaining_--;
    if (vibration_pulses_remaining_ > 0) {
      // Pause between pulses
      vibration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(current_vibration_duration_ms_),
        std::bind(&JoystickNode::vibration_timer_callback, this));
    }
  }
}

void JoystickNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  // Toggle logic (rising edge)
  const bool toggle_button_pressed = get_button(*msg, toggle_button_);
  if (toggle_button_pressed && !prev_toggle_button_pressed_) {
    // Send zero to the current topic before switching to avoid stale commands
    publish_zero_command();

    use_roscco_topic_ = !use_roscco_topic_;
    RCLCPP_INFO(
      this->get_logger(),
      "Toggled output topic to: %s",
      use_roscco_topic_ ? "/joystick/roscco" : "/joystick/ackermann");
    vibrate(use_roscco_topic_ ? 2 : 1, toggle_vibration_duration_ms_);
  }
  prev_toggle_button_pressed_ = toggle_button_pressed;

  // Arming logic (rising edge)
  const bool arming_button_pressed = get_button(*msg, arming_button_);
  if (arming_button_pressed && !prev_arming_button_pressed_) {
    if (arm_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      // if is_armed_ is true, we want to disarm (false)
      // if is_armed_ is false, we want to arm (true)
      const bool target_state = !is_armed_;
      request->data = target_state;

      RCLCPP_INFO(this->get_logger(), "Requesting arm state: %s", target_state ? "ARM" : "DISARM");

      using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
      auto response_received_callback = [this, target_state](ServiceResponseFuture future) {
        try {
          auto result = future.get();
          if (result->success) {
            if (target_state) {
              // Turing ON (Arming) -> Double vibrate
              vibrate(2, toggle_vibration_duration_ms_);  // Use default duration or maybe shorter? keeping default
            } else {
              // Turing OFF (Disarming) -> Long vibrate
              vibrate(1, 500);  // 500ms long pulse
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "Arming service failed: %s", result->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
      };
      arm_client_->async_send_request(request, response_received_callback);
    } else {
      RCLCPP_WARN(this->get_logger(), "Arming service not ready!");
    }
  }
  prev_arming_button_pressed_ = arming_button_pressed;

  // Safety gating
  // enable must be held
  const bool enable_pressed = get_axis(*msg, enable_axis_) <= -0.9;

  // If enable is not held, fully disarm and stop.
  if (!enable_pressed) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Safety not met (enable_axis=false) -> publishing zero command");
    publish_neutral_state(true);
    return;
  }

  // Read + process axes
  double steer = get_axis(*msg, steering_axis_);
  double throttle = get_axis(*msg, throttle_axis_);

  if (invert_steering_) {
    steer = -steer;
  }

  if (invert_throttle_) {
    throttle = -throttle;
  }

  if (use_roscco_topic_) {
    // Scale to physical commands
    const double steering_angle = std::clamp(steer, -1.0, 1.0) * roscco_max_steering_angle_;
    const double speed = std::clamp(throttle, -1.0, 1.0) * roscco_max_speed_;

    roscco_msg::msg::Roscco cmd_stamped;
    cmd_stamped.header.stamp = this->now();
    cmd_stamped.steering = steering_angle;
    cmd_stamped.forward = speed;
    roscco_joystick_pub_->publish(cmd_stamped);
  } else {
    // Scale to physical commands
    const double steering_angle = std::clamp(steer, -1.0, 1.0) * ackermann_max_steering_angle_;
    const double speed = std::clamp(throttle, -1.0, 1.0) * ackermann_max_speed_;
    ackermann_msgs::msg::AckermannDriveStamped cmd_stamped;
    cmd_stamped.header.stamp = this->now();
    cmd_stamped.drive.steering_angle = steering_angle;
    cmd_stamped.drive.speed = speed;
    ackermann_drive_stamped_pub_->publish(cmd_stamped);
  }

  publish_state(use_roscco_topic_ ? JoystickNode::JoystickState::ROSSCO : JoystickNode::JoystickState::ACKERMANN);
  publish_idle_state(false);  // Joystick is active, so NOT idle
}

}  // namespace joystick_node

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_node::JoystickNode)
