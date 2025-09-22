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

#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <interfacing_msgs/msg/vehicle_status.hpp>

#include "state_estimation/wheel_odometry_core.hpp"

wato::world_modeling::state_estimation::WheelOdometryCore::WheelOdometryCore(double wheel_base)
: wheel_base_(wheel_base) {}

void wato::world_modeling::state_estimation::WheelOdometryCore::setWheelBase(double wheel_base)
{
  wheel_base_ = wheel_base;
}

void wato::world_modeling::state_estimation::WheelOdometryCore::computeOdometry(
  const interfacing_msgs::msg::VehicleStatus::SharedPtr & msg, nav_msgs::msg::Odometry & odom_msg)
{
  // Get current vehicle speed and steering angle
  double speed = msg->speed;  // in m/s
  double steering_angle = msg->steering_angle;  // in radians

  // Update bicycle model based on received data
  double _forward_velocity = speed;  // vehicle moves forward at the speed
  double _lateral_velocity = 0.0;  // vehicle cannot move laterally
  double _angular_velocity = -(speed / wheel_base_) * tan(steering_angle);

  // Insert data into odometry message
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = _forward_velocity;
  odom_msg.twist.twist.linear.y = _lateral_velocity;
  odom_msg.twist.twist.angular.z = _angular_velocity;
}
