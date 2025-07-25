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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"


class WheelOdometry : public rclcpp::Node
{
public:
  WheelOdometry();

private:
  void bicycleModel();
  void vehicleStatusCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_sub;

  // Carla sim data
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr vehicle_status_sub_;

  // To initialize with Carla starting pose and theta
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr init_sub_;
  void initializeOdomFromCarla(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};

  double wheel_base_;
  double max_steer_angle_;

  double left_wheel_speed;
  double right_wheel_speed;

  double velocity_;
  double steering_angle_;
  double x_{0.0}, y_{0.0}, theta_{0.0};
};

#endif
