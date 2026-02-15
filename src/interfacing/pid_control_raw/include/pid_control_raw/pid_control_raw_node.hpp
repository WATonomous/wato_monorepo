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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pid_control_raw/pid.hpp"

namespace pid_control_raw
{

/**
 * @brief Node for dual-loop PID control of steering and velocity.
 *        Uses a self-contained PID implementation (no control_toolbox).
 */
class PidControlRawNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PidControlRawNode(const rclcpp::NodeOptions & options);

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void steering_feedback_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg);
  void velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void control_loop();

  /**
   * @brief Read PID gains from ROS parameters for a given prefix.
   */
  Pid::Gains load_pid_gains(const std::string & prefix);

  // Subscriptions
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<roscco_msg::msg::SteeringAngle>::SharedPtr steering_meas_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_meas_sub_;

  // Publisher
  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr roscco_pub_;

  // PID controllers
  Pid steering_pid_;
  Pid velocity_pid_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state
  double steering_setpoint_{0.0};
  double steering_meas_{0.0};

  double velocity_setpoint_{0.0};
  double velocity_meas_{0.0};

  bool ackermann_received_{false};
  bool steering_meas_received_{false};
  bool velocity_meas_received_{false};

  rclcpp::Time last_time_;
  double steering_output_prev_{0.0};
  double steering_slew_rate_{2.0};
};

}  // namespace pid_control_raw
