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

#include "pid_control_raw/pid_control_raw_node.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace pid_control_raw
{

PidControlRawNode::PidControlRawNode(const rclcpp::NodeOptions & options)
: LifecycleNode("pid_control_raw_node", options)
{
  // Declare top-level parameters
  this->declare_parameter<double>("update_rate", 100.0);
  this->declare_parameter<double>("steering_slew_rate", 100.0);

  // Declare steering PID parameters
  this->declare_parameter<double>("steering_pid.p", 1.0);
  this->declare_parameter<double>("steering_pid.i", 0.0);
  this->declare_parameter<double>("steering_pid.d", 0.0);
  this->declare_parameter<double>("steering_pid.i_clamp_max", 0.56);
  this->declare_parameter<double>("steering_pid.i_clamp_min", -0.56);
  this->declare_parameter<double>("steering_pid.u_clamp_max", 0.56);
  this->declare_parameter<double>("steering_pid.u_clamp_min", -0.56);

  // Declare velocity PID parameters
  this->declare_parameter<double>("velocity_pid.p", 1.0);
  this->declare_parameter<double>("velocity_pid.i", 0.1);
  this->declare_parameter<double>("velocity_pid.d", 0.01);
  this->declare_parameter<double>("velocity_pid.i_clamp_max", 0.5);
  this->declare_parameter<double>("velocity_pid.i_clamp_min", -0.5);
  this->declare_parameter<double>("velocity_pid.u_clamp_max", 1.0);
  this->declare_parameter<double>("velocity_pid.u_clamp_min", -1.0);

  RCLCPP_INFO(this->get_logger(), "PidControlRawNode created (unconfigured)");
}

Pid::Gains PidControlRawNode::load_pid_gains(const std::string & prefix)
{
  Pid::Gains gains;
  gains.p = this->get_parameter(prefix + ".p").as_double();
  gains.i = this->get_parameter(prefix + ".i").as_double();
  gains.d = this->get_parameter(prefix + ".d").as_double();
  gains.i_clamp_max = this->get_parameter(prefix + ".i_clamp_max").as_double();
  gains.i_clamp_min = this->get_parameter(prefix + ".i_clamp_min").as_double();
  gains.u_clamp_max = this->get_parameter(prefix + ".u_clamp_max").as_double();
  gains.u_clamp_min = this->get_parameter(prefix + ".u_clamp_min").as_double();
  return gains;
}

PidControlRawNode::CallbackReturn PidControlRawNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Load PID gains from parameters
  steering_pid_.set_gains(load_pid_gains("steering_pid"));
  steering_pid_.reset();

  velocity_pid_.set_gains(load_pid_gains("velocity_pid"));
  velocity_pid_.reset();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann", rclcpp::QoS(10), std::bind(&PidControlRawNode::ackermann_callback, this, std::placeholders::_1));

  steering_meas_sub_ = this->create_subscription<roscco_msg::msg::SteeringAngle>(
    "steering_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlRawNode::steering_feedback_callback, this, std::placeholders::_1));

  velocity_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "velocity_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlRawNode::velocity_feedback_callback, this, std::placeholders::_1));

  // Publisher
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

PidControlRawNode::CallbackReturn PidControlRawNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  double update_rate = this->get_parameter("update_rate").as_double();
  steering_slew_rate_ = this->get_parameter("steering_slew_rate").as_double();

  // Re-load gains in case they were changed between configure and activate
  steering_pid_.set_gains(load_pid_gains("steering_pid"));
  velocity_pid_.set_gains(load_pid_gains("velocity_pid"));

  // Start control loop timer
  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&PidControlRawNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Activated - control loop running at %.1f Hz", update_rate);
  return CallbackReturn::SUCCESS;
}

PidControlRawNode::CallbackReturn PidControlRawNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

PidControlRawNode::CallbackReturn PidControlRawNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  timer_.reset();
  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  roscco_pub_.reset();

  // Reset state
  steering_pid_.reset();
  velocity_pid_.reset();
  steering_setpoint_ = 0.0;
  steering_meas_ = 0.0;
  velocity_setpoint_ = 0.0;
  velocity_meas_ = 0.0;
  ackermann_received_ = false;
  steering_meas_received_ = false;
  velocity_meas_received_ = false;

  return CallbackReturn::SUCCESS;
}

PidControlRawNode::CallbackReturn PidControlRawNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  roscco_pub_.reset();

  return CallbackReturn::SUCCESS;
}

void PidControlRawNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  steering_setpoint_ = msg->drive.steering_angle;
  velocity_setpoint_ = msg->drive.speed;
  ackermann_received_ = true;
}

void PidControlRawNode::steering_feedback_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg)
{
  steering_meas_ = msg->angle;
  steering_meas_received_ = true;
}

void PidControlRawNode::velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  velocity_meas_ = msg->data;
  velocity_meas_received_ = true;
}

void PidControlRawNode::control_loop()
{
  rclcpp::Time now = this->now();
  rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.nanoseconds() <= 0) {
    return;
  }

  if (!ackermann_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for ackermann setpoint...");
    return;
  }

  double steering_command = 0.0;
  double velocity_command = 0.0;

  // Compute Steering Command (Torque)
  if (steering_meas_received_) {
    double steering_error = steering_setpoint_ - steering_meas_;
    steering_command = steering_pid_.compute(steering_error, dt.seconds());
    if (steering_command > steering_output_prev_) {
      steering_command = std::min(steering_command, steering_output_prev_ + steering_slew_rate_ * dt.seconds());
    } else if (steering_command < steering_output_prev_) {
      steering_command = std::max(steering_command, steering_output_prev_ - steering_slew_rate_ * dt.seconds());
    }
    steering_output_prev_ = steering_command;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for steering feedback...");
  }

  // Compute Velocity Command
  if (velocity_meas_received_) {
    double velocity_error = velocity_setpoint_ - velocity_meas_;
    velocity_command = velocity_pid_.compute(velocity_error, dt.seconds());
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for velocity feedback...");
  }

  // Publish Roscco message
  roscco_msg::msg::Roscco msg;
  msg.header.stamp = now;
  msg.steering = static_cast<float>(steering_command);
  msg.forward = static_cast<float>(velocity_command);

  roscco_pub_->publish(msg);
}

}  // namespace pid_control_raw

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control_raw::PidControlRawNode)
