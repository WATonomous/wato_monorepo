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

#include "pid_control/ackermann_smoother_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>

namespace pid_control
{

// Acceleration-limited smoothing with braking.
// Accelerates toward target, then decelerates to arrive with zero velocity.
static double smooth_step(
  double current, double & rate, double target,
  double max_accel, double dt)
{
  double error = target - current;

  // Compute the braking distance at current rate: d = v^2 / (2*a)
  // If we're within braking distance, decelerate to stop at the target.
  double braking_dist = (rate * rate) / (2.0 * max_accel);
  double accel;

  if (std::abs(error) < 1e-6 && std::abs(rate) < 1e-6) {
    // Already at target and stopped
    rate = 0.0;
    return target;
  }

  bool moving_toward = (error * rate >= 0.0);

  if (!moving_toward && std::abs(rate) > 1e-6) {
    // Moving away from target — brake hard
    accel = (rate > 0.0) ? -max_accel : max_accel;
  } else if (std::abs(error) <= braking_dist) {
    // Within braking distance — decelerate to arrive at target
    accel = (rate > 0.0) ? -max_accel : max_accel;
  } else {
    // Accelerate toward target
    accel = (error > 0.0) ? max_accel : -max_accel;
  }

  rate += accel * dt;
  double next = current + rate * dt;

  // Snap to target if we've crossed it and rate is small
  if ((current <= target && next >= target && rate >= 0.0) ||
      (current >= target && next <= target && rate <= 0.0))
  {
    if (std::abs(rate) <= max_accel * dt * 2.0) {
      rate = 0.0;
      return target;
    }
  }

  return next;
}

AckermannSmootherNode::AckermannSmootherNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ackermann_smoother_node", options)
{
  this->declare_parameter<double>("max_steering_accel", 2.0);
  this->declare_parameter<double>("max_speed_accel", 2.0);
  this->declare_parameter<double>("publish_rate", 50.0);

  RCLCPP_INFO(this->get_logger(), "AckermannSmootherNode created (unconfigured)");
}

AckermannSmootherNode::CallbackReturn AckermannSmootherNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  max_steering_accel_ = this->get_parameter("max_steering_accel").as_double();
  max_speed_accel_ = this->get_parameter("max_speed_accel").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  input_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_in",
    rclcpp::QoS(10),
    std::bind(&AckermannSmootherNode::input_callback, this, std::placeholders::_1));

  output_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_out",
    rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

AckermannSmootherNode::CallbackReturn AckermannSmootherNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AckermannSmootherNode::publish_loop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Activated - publishing at %.1f Hz, max_steering_accel=%.3f rad/s^2, max_speed_accel=%.3f m/s^3",
    publish_rate_, max_steering_accel_, max_speed_accel_);
  return CallbackReturn::SUCCESS;
}

AckermannSmootherNode::CallbackReturn AckermannSmootherNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  return CallbackReturn::SUCCESS;
}

AckermannSmootherNode::CallbackReturn AckermannSmootherNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  timer_.reset();
  input_sub_.reset();
  output_pub_.reset();
  current_steering_ = 0.0;
  steering_rate_ = 0.0;
  current_speed_ = 0.0;
  speed_rate_ = 0.0;
  target_steering_ = 0.0;
  target_speed_ = 0.0;
  input_received_ = false;
  return CallbackReturn::SUCCESS;
}

AckermannSmootherNode::CallbackReturn AckermannSmootherNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  input_sub_.reset();
  output_pub_.reset();
  return CallbackReturn::SUCCESS;
}

void AckermannSmootherNode::input_callback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  target_steering_ = msg->drive.steering_angle;
  target_speed_ = msg->drive.speed;
  input_received_ = true;
}

void AckermannSmootherNode::publish_loop()
{
  if (!input_received_) {
    return;
  }

  double dt = 1.0 / publish_rate_;

  current_steering_ = smooth_step(
    current_steering_, steering_rate_, target_steering_, max_steering_accel_, dt);

  current_speed_ = smooth_step(
    current_speed_, speed_rate_, target_speed_, max_speed_accel_, dt);

  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.stamp = this->now();
  msg.drive.steering_angle = static_cast<float>(current_steering_);
  msg.drive.speed = static_cast<float>(current_speed_);

  output_pub_->publish(msg);
}

}  // namespace pid_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_control::AckermannSmootherNode)
