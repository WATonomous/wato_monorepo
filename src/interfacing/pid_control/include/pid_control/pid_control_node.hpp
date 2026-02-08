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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <std_msgs/msg/float64.hpp>

namespace pid_control
{

/**
 * @brief Node for dual-loop PID control of steering and velocity.
 */
class PidControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PidControlNode(const rclcpp::NodeOptions & options);

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Callback for incoming Ackermann setpoints.
   *
   * @param msg The desired steering angle and speed.
   */
  void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  /**
   * @brief Callback for steering angle measurement feedback.
   *
   * @param msg The current steering angle in degrees.
   */
  void steering_feedback_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg);

  /**
   * @brief Callback for velocity measurement feedback.
   *
   * @param msg The current vehicle speed in m/s.
   */
  void velocity_feedback_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief The main control loop that computes and publishes PID commands.
   */
  void control_loop();

  // Subscriptions
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<roscco_msg::msg::SteeringAngle>::SharedPtr steering_meas_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_meas_sub_;

  // Publisher
  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr roscco_pub_;

  // PID
  std::shared_ptr<control_toolbox::PidROS> steering_pid_ros_;
  std::shared_ptr<control_toolbox::PidROS> velocity_pid_ros_;

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
};

}  // namespace pid_control
