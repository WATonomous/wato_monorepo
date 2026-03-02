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

#include "dummy_ackermann/ackermann_square_wave_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace dummy_ackermann
{

AckermannSquareWaveNode::AckermannSquareWaveNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ackermann_square_wave", options)
{
  this->declare_parameter<double>("period", 2.0);
  this->declare_parameter<double>("amplitude", 0.1);
  this->declare_parameter<double>("publish_rate", 50.0);

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AckermannSquareWaveNode::on_set_parameters, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AckermannSquareWaveNode created");
}

AckermannSquareWaveNode::CallbackReturn AckermannSquareWaveNode::on_configure(const rclcpp_lifecycle::State &)
{
  period_ = this->get_parameter("period").as_double();
  amplitude_ = this->get_parameter("amplitude").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  if (period_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Period must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Publish rate must be > 0");
    return CallbackReturn::FAILURE;
  }

  pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann", 10);

  RCLCPP_INFO(
    this->get_logger(), "Configured: period=%.2fs amplitude=%.2f rad rate=%.1f Hz", period_, amplitude_, publish_rate_);

  return CallbackReturn::SUCCESS;
}

AckermannSquareWaveNode::CallbackReturn AckermannSquareWaveNode::on_activate(const rclcpp_lifecycle::State &)
{
  start_time_ = this->now();

  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_));

  timer_ = this->create_wall_timer(period_ns, std::bind(&AckermannSquareWaveNode::timer_callback, this));

  pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

AckermannSquareWaveNode::CallbackReturn AckermannSquareWaveNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

AckermannSquareWaveNode::CallbackReturn AckermannSquareWaveNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

AckermannSquareWaveNode::CallbackReturn AckermannSquareWaveNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(this->get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void AckermannSquareWaveNode::timer_callback()
{
  auto now = this->now();
  double elapsed = (now - start_time_).seconds();

  // Square wave logic: toggle every period / 2
  // fmod returns remainder of elapsed / period_
  // If elapsed % period_ < period_ / 2, steering = amplitude
  // Else steering = -amplitude
  double steering = (std::fmod(elapsed, period_) < (period_ / 2.0)) ? amplitude_ : -amplitude_;

  auto msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
  msg->header.stamp = now;
  msg->header.frame_id = "base_link";
  msg->drive.steering_angle = static_cast<float>(steering);
  // Leave speed unset to avoid interfering with velocity control

  pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult AckermannSquareWaveNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "period") {
      double val = param.as_double();
      if (val <= 0.0) {
        result.successful = false;
        result.reason = "Period must be > 0";
      } else {
        period_ = val;
        RCLCPP_INFO(this->get_logger(), "Updated period to %.2f", period_);
      }
    } else if (param.get_name() == "amplitude") {
      amplitude_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated amplitude to %.2f", amplitude_);
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
          auto period_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_));
          timer_.reset();
          timer_ = this->create_wall_timer(period_ns, std::bind(&AckermannSquareWaveNode::timer_callback, this));
        }
      }
    }
  }

  return result;
}

}  // namespace dummy_ackermann

RCLCPP_COMPONENTS_REGISTER_NODE(dummy_ackermann::AckermannSquareWaveNode)
