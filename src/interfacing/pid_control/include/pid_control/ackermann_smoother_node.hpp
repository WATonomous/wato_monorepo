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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace pid_control
{

class AckermannSmootherNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AckermannSmootherNode(const rclcpp::NodeOptions & options);

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void input_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void publish_loop();

  // Subscription and publisher
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr input_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Acceleration limiting parameters
  double max_steering_accel_{2.0};  // rad/s^2 - max change in steering rate
  double max_speed_accel_{2.0};  // m/s^3 - max change in speed rate (jerk)
  double publish_rate_{50.0};  // Hz

  // Current smoothed state (position + velocity)
  double current_steering_{0.0};
  double steering_rate_{0.0};

  double current_speed_{0.0};
  double speed_rate_{0.0};

  // Target setpoints from input
  double target_steering_{0.0};
  double target_speed_{0.0};

  bool input_received_{false};
};

}  // namespace pid_control
