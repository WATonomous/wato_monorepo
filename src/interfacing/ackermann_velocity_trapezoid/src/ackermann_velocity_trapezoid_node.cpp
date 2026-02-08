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

#include "ackermann_velocity_trapezoid/ackermann_velocity_trapezoid_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ackermann_velocity_trapezoid
{

AckermannVelocityTrapezoidNode::AckermannVelocityTrapezoidNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ackermann_velocity_trapezoid", options)
{
  this->declare_parameter<double>("target_velocity", 1.0);
  this->declare_parameter<double>("rise_time", 2.0);
  this->declare_parameter<double>("hold_time", 3.0);
  this->declare_parameter<double>("ramp_down_time", 2.0);
  this->declare_parameter<double>("publish_rate", 50.0);

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AckermannVelocityTrapezoidNode::on_set_parameters, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AckermannVelocityTrapezoidNode created");
}

AckermannVelocityTrapezoidNode::CallbackReturn AckermannVelocityTrapezoidNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  target_velocity_ = this->get_parameter("target_velocity").as_double();
  rise_time_ = this->get_parameter("rise_time").as_double();
  hold_time_ = this->get_parameter("hold_time").as_double();
  ramp_down_time_ = this->get_parameter("ramp_down_time").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  if (rise_time_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Rise time must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (hold_time_ < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Hold time must be >= 0");
    return CallbackReturn::FAILURE;
  }

  if (ramp_down_time_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Ramp down time must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Publish rate must be > 0");
    return CallbackReturn::FAILURE;
  }

  pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Configured: target_velocity=%.2f m/s, rise_time=%.2fs, hold_time=%.2fs, "
    "ramp_down_time=%.2fs, rate=%.1f Hz",
    target_velocity_, rise_time_, hold_time_, ramp_down_time_, publish_rate_);

  return CallbackReturn::SUCCESS;
}

AckermannVelocityTrapezoidNode::CallbackReturn AckermannVelocityTrapezoidNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  start_time_ = this->now();

  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));

  timer_ = this->create_wall_timer(
    period_ns, std::bind(&AckermannVelocityTrapezoidNode::timer_callback, this));

  pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

AckermannVelocityTrapezoidNode::CallbackReturn AckermannVelocityTrapezoidNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

AckermannVelocityTrapezoidNode::CallbackReturn AckermannVelocityTrapezoidNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

AckermannVelocityTrapezoidNode::CallbackReturn AckermannVelocityTrapezoidNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void AckermannVelocityTrapezoidNode::timer_callback()
{
  auto now = this->now();
  double elapsed = (now - start_time_).seconds();

  // Calculate total cycle period
  double cycle_period = rise_time_ + hold_time_ + ramp_down_time_;

  // Get position within current cycle
  double cycle_time = std::fmod(elapsed, cycle_period);

  double velocity = 0.0;

  // Trapezoid ramp logic:
  // Phase 1: Rise (0 to rise_time_) - linear ramp from 0 to target_velocity
  if (cycle_time < rise_time_) {
    velocity = (cycle_time / rise_time_) * target_velocity_;
  } else if (cycle_time < rise_time_ + hold_time_) {
    // Phase 2: Hold - constant at target_velocity
    velocity = target_velocity_;
  } else {
    // Phase 3: Ramp down - linear ramp from target_velocity to 0
    double ramp_down_elapsed = cycle_time - (rise_time_ + hold_time_);
    velocity = target_velocity_ * (1.0 - (ramp_down_elapsed / ramp_down_time_));
  }

  auto msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
  msg->header.stamp = now;
  msg->header.frame_id = "base_link";
  msg->drive.speed = static_cast<float>(velocity);
  // Leave steering_angle unset to avoid interfering with steering control

  pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult AckermannVelocityTrapezoidNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "target_velocity") {
      target_velocity_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated target_velocity to %.2f", target_velocity_);
    } else if (param.get_name() == "rise_time") {
      double val = param.as_double();
      if (val <= 0.0) {
        result.successful = false;
        result.reason = "Rise time must be > 0";
      } else {
        rise_time_ = val;
        RCLCPP_INFO(this->get_logger(), "Updated rise_time to %.2f", rise_time_);
      }
    } else if (param.get_name() == "hold_time") {
      double val = param.as_double();
      if (val < 0.0) {
        result.successful = false;
        result.reason = "Hold time must be >= 0";
      } else {
        hold_time_ = val;
        RCLCPP_INFO(this->get_logger(), "Updated hold_time to %.2f", hold_time_);
      }
    } else if (param.get_name() == "ramp_down_time") {
      double val = param.as_double();
      if (val <= 0.0) {
        result.successful = false;
        result.reason = "Ramp down time must be > 0";
      } else {
        ramp_down_time_ = val;
        RCLCPP_INFO(this->get_logger(), "Updated ramp_down_time to %.2f", ramp_down_time_);
      }
    } else if (param.get_name() == "publish_rate") {
      double val = param.as_double();
      if (val <= 0.0) {
        result.successful = false;
        result.reason = "Publish rate must be > 0";
      } else {
        publish_rate_ = val;
        RCLCPP_INFO(this->get_logger(), "Updated publish_rate to %.2f", publish_rate_);
        // Update timer if active
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_));
          timer_.reset();
          timer_ = this->create_wall_timer(
            period_ns, std::bind(&AckermannVelocityTrapezoidNode::timer_callback, this));
        }
      }
    }
  }

  return result;
}

}  // namespace ackermann_velocity_trapezoid

RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_velocity_trapezoid::AckermannVelocityTrapezoidNode)
