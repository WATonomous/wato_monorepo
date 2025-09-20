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

#ifndef ODOM_HPP
#define ODOM_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common_msgs/msg/vehicle_status.hpp"

class WheelOdometry : public rclcpp::Node
{
public:
  WheelOdometry();

private:
  // Vehicle Status Subscriber
  rclcpp::Subscription<common_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  void vehicleStatusCallback(const common_msgs::msg::VehicleStatus::SharedPtr msg);

  // Wheel Odometry Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_publisher_;

  // Other member variables
  double wheel_base_;
};

#endif
