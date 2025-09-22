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

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <interfacing_msgs/msg/vehicle_status.hpp>

#include "state_estimation/wheel_odometry_core.hpp"

/**
 * @brief Tests if the sign of the angular velocity is correct.
 */
TEST(WheelOdometryCoreTest, CheckAngularVelocitySign) {
  // Initialize WheelOdometryCore with a wheel base of 2.5 meters
  wato::world_modeling::state_estimation::WheelOdometryCore odometry_core(2.5);

  // Create a mock VehicleStatus message
  auto vehicle_status = std::make_shared<interfacing_msgs::msg::VehicleStatus>();
  vehicle_status->speed = 10.0;    // 10 m/s
  vehicle_status->steering_angle = 0.1;    // 0.1 radians

  // Create an Odometry message to populate
  nav_msgs::msg::Odometry odom_msg;

  // Compute odometry
  odometry_core.computeOdometry(vehicle_status, odom_msg);

  // The angular velocity (yaw) should be negative for a positive steering angle
  EXPECT_LT(odom_msg.twist.twist.angular.z, 0.0);

  // Update mock VehicleStatus message with negative steering angle
  vehicle_status->steering_angle = -0.1;    // -0.1 radians

  // Compute odometry again
  odometry_core.computeOdometry(vehicle_status, odom_msg);

  // The angular velocity (yaw) should be positive for a negative steering angle
  EXPECT_GT(odom_msg.twist.twist.angular.z, 0.0);
}
