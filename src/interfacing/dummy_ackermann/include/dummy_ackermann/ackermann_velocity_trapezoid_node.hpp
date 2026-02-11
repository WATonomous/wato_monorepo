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

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dummy_ackermann
{

class AckermannVelocityTrapezoidNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AckermannVelocityTrapezoidNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void timer_callback();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  double target_velocity_;  // m/s
  double rise_time_;  // seconds
  double hold_time_;  // seconds
  double ramp_down_time_;  // seconds
  double publish_rate_;  // Hz

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rclcpp::Time start_time_;
};

}  // namespace dummy_ackermann
