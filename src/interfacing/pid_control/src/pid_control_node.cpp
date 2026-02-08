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

#include "pid_control/pid_control_node.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace pid_control
{

PidControlNode::PidControlNode(const rclcpp::NodeOptions & options)
: LifecycleNode("pid_control_node", options)
{
  // Declare parameters only - do not read or create resources yet
  this->declare_parameter<double>("update_rate", 100.0);

  RCLCPP_INFO(this->get_logger(), "PidControlNode created (unconfigured)");
}

PidControlNode::CallbackReturn PidControlNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");


  // Initialize Steering PID
  steering_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "steering_pid",
    "/steering_pid_state",
    true);
  steering_pid_ros_->initialize_from_ros_parameters();

  // Initialize Velocity PID
  velocity_pid_ros_ = std::make_shared<control_toolbox::PidROS>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_topics_interface(),
    "velocity_pid",
    "/velocity_pid_state",
    true);
  velocity_pid_ros_->initialize_from_ros_parameters();

  // Subscriptions
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann", rclcpp::QoS(10), std::bind(&PidControlNode::ackermann_callback, this, std::placeholders::_1));

  steering_meas_sub_ = this->create_subscription<roscco_msg::msg::SteeringAngle>(
    "steering_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlNode::steering_feedback_callback, this, std::placeholders::_1));

  velocity_meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "velocity_feedback",
    rclcpp::QoS(10),
    std::bind(&PidControlNode::velocity_feedback_callback, this, std::placeholders::_1));

  // Publisher
  roscco_pub_ = this->create_publisher<roscco_msg::msg::Roscco>("roscco", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

PidControlNode::CallbackReturn PidControlNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  double update_rate = this->get_parameter("update_rate").as_double();

  // Start control loop timer
  last_time_ = this->now();
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&PidControlNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Activated - control loop running at %.1f Hz", update_rate);
  return CallbackReturn::SUCCESS;
}

PidControlNode::CallbackReturn PidControlNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  // Stop the timer
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

PidControlNode::CallbackReturn PidControlNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  // Reset all resources
  timer_.reset();
  ackermann_sub_.reset();
  steering_meas_sub_.reset();
  velocity_meas_sub_.reset();
  roscco_pub_.reset();
  steering_pid_ros_.reset();
  velocity_pid_ros_.reset();

  // Reset state
  steering_setpoint_ = 0.0;
  steering_meas_ = 0.0;
  velocity_setpoint_ = 0.0;
  velocity_meas_ = 0.0;
  ackermann_received_ = false;
  steering_meas_received_ = false;
  velocity_meas_received_ = false;

  return CallbackReturn::SUCCESS;
}

PidControlNode::CallbackReturn PidControlNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
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
  steering_pid_ros_.reset();
  velocity_pid_ros_.reset();

  return CallbackReturn::SUCCESS;
}

void PidControlNode::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  steering_setpoint_ = msg->drive.steering_angle;
  velocity_setpoint_ = msg->drive.speed;
  ackermann_received_ = true;
}

void PidControlNode::steering_feedback_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg)
{
  steering_meas_ = msg->angle;
  steering_meas_received_ = true;
}

void PidControlNode::velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  velocity_meas_ = msg->data;
  velocity_meas_received_ = true;
}

void PidControlNode::control_loop()
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

  /*
  if (!steering_meas_received_ || !velocity_meas_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Waiting for feedback signals (steering: %s, velocity: %s)...",
      steering_meas_received_ ? "OK" : "MISSING",
      velocity_meas_received_ ? "OK" : "MISSING");
    return;
  }
  */

  double steering_command = 0.0;
  double velocity_command = 0.0;

  // Compute Steering Command (Torque)
  if (steering_meas_received_) {
    double steering_error = steering_setpoint_ - steering_meas_;
    steering_command = steering_pid_ros_->compute_command(steering_error, dt);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for steering feedback...");
  }

  // Compute Velocity Command
  if (velocity_meas_received_) {
    double velocity_error = velocity_setpoint_ - velocity_meas_;
    velocity_command = velocity_pid_ros_->compute_command(velocity_error, dt);
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

}  // namespace pid_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::PidControlNode)
