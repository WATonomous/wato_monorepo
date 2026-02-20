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

#ifndef CAR_VELOCITY_FEEDBACK__CAR_VELOCITY_FEEDBACK_NODE_HPP_
#define CAR_VELOCITY_FEEDBACK__CAR_VELOCITY_FEEDBACK_NODE_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>
#include <std_msgs/msg/float64.hpp>

namespace car_velocity_feedback
{

/**
 * @brief Node that estimates vehicle body velocity from wheel speeds and steering angle.
 *
 * Uses the Ackermann bicycle model approximation: v_body = v_front_avg * cos(steering_angle).
 */
class CarVelocityFeedbackNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CarVelocityFeedbackNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configures the node, including parameters and subscribers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  /**
   * @brief Activates the node, enabling publishers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  /**
   * @brief Deactivates the node, disabling publishers.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

  /**
   * @brief Cleans up the node, releasing resources.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  /**
   * @brief Shuts down the node.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  /**
   * @brief Callback for wheel speed data.
   *
   * Calculates body velocity and publishes it if the node is active.
   * @param msg The wheel speeds message containing NE, NW, SE, SW speeds.
   */
  void wheel_speeds_callback(const roscco_msg::msg::WheelSpeeds::SharedPtr msg);

  /**
   * @brief Callback for steering angle data.
   *
   * Updates the current steering angle used for velocity projection.
   * @param msg The steering angle message.
   */
  void steering_angle_callback(const roscco_msg::msg::SteeringAngle::SharedPtr msg);

  rclcpp::Subscription<roscco_msg::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
  rclcpp::Subscription<roscco_msg::msg::SteeringAngle>::SharedPtr steering_angle_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr body_velocity_pub_;

  double current_steering_angle_rad_;
  bool has_steering_angle_;
};

}  // namespace car_velocity_feedback

#endif  // CAR_VELOCITY_FEEDBACK__CAR_VELOCITY_FEEDBACK_NODE_HPP_
