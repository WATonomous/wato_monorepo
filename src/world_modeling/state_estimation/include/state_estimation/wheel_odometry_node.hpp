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

#ifndef STATE_ESTIMATION__WHEEL_ODOMETRY_NODE_HPP_
#define STATE_ESTIMATION__WHEEL_ODOMETRY_NODE_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "interfacing_msgs/msg/vehicle_status.hpp"
#include "state_estimation/wheel_odometry_core.hpp"

/**
 * @class WheelOdometryNode
 * @brief A ROS 2 node for handling wheel odometry computation and publishing.
 */
class WheelOdometryNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor to initialize the WheelOdometryNode.
   */
  WheelOdometryNode();

private:
  /**
   * @brief Callback function for the vehicle status subscriber.
   * @param msg Shared pointer to the received vehicle status message.
   */
  void vehicleStatusCallback(const interfacing_msgs::msg::VehicleStatus::SharedPtr msg);

  /**
   * @brief Subscriber for vehicle status messages.
   */
  rclcpp::Subscription<interfacing_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

  /**
   * @brief Publisher for wheel odometry messages.
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_publisher_;

  /**
   * @brief Core logic handler for wheel odometry computation.
   */
  wato::world_modeling::state_estimation::WheelOdometryCore wheel_odometry_;
};

#endif  // STATE_ESTIMATION__WHEEL_ODOMETRY_NODE_HPP_
