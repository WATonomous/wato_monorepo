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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <std_msgs/msg/float64.hpp>

namespace pid_control
{

class PidControlNode : public rclcpp::Node
{
public:
  explicit PidControlNode(const rclcpp::NodeOptions & options);

private:
  void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void feedback_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void control_loop();

  // Subscriptions
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr feedback_sub_;

  // Publisher
  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr roscco_pub_;

  // PID
  std::shared_ptr<control_toolbox::PidROS> pid_ros_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state
  double setpoint_{0.0};
  double feedback_{0.0};
  bool setpoint_received_{false};
  bool feedback_received_{false};

  rclcpp::Time last_time_;
};

}  // namespace pid_control
