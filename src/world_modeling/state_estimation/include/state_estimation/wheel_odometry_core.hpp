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

#ifndef STATE_ESTIMATION__WHEEL_ODOMETRY_CORE_HPP_
#define STATE_ESTIMATION__WHEEL_ODOMETRY_CORE_HPP_

#include <nav_msgs/msg/odometry.hpp>

#include "interfacing_msgs/msg/vehicle_status.hpp"

namespace wato::world_modeling::state_estimation
{

/**
 * @class WheelOdometryCore
 * @brief A class to compute odometry based on vehicle status and wheel base.
 */
class WheelOdometryCore
{
public:
  /**
   * @brief Constructor to initialize the WheelOdometryCore with a given wheel base.
   * @param wheel_base The distance between the front and rear axles of the vehicle.
   */
  explicit WheelOdometryCore(double wheel_base);

  /**
   * @brief Computes the odometry message based on the vehicle status.
   * @param msg Shared pointer to the vehicle status message.
   * @param odom_msg Reference to the odometry message to be populated.
   */
  void computeOdometry(const interfacing_msgs::msg::VehicleStatus::SharedPtr & msg, nav_msgs::msg::Odometry & odom_msg);

  /**
   * @brief Sets the wheel base of the vehicle.
   * @param wheel_base The new wheel base value.
   */
  void setWheelBase(double wheel_base);

private:
  /**
   * @brief The distance between the front and rear axles of the vehicle.
   */
  double wheel_base_;
};

}  // namespace wato::world_modeling::state_estimation

#endif  // STATE_ESTIMATION__WHEEL_ODOMETRY_CORE_HPP_
