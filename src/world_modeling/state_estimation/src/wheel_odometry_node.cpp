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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <interfacing_msgs/msg/vehicle_status.hpp>

#include "state_estimation/wheel_odometry_node.hpp"
#include "state_estimation/wheel_odometry_core.hpp"

WheelOdometryNode::WheelOdometryNode()
: Node("wheel_odometry"), wheel_odometry_(2.875)  // default wheel base in meters
{
  // Input and Output Topic Names
  this->declare_parameter<std::string>("vehicle_status_topic", std::string("/vehicle_status"));
  this->declare_parameter<std::string>(
    "wheel_odometry_output_topic",
    std::string("/state_estimation/wheel_odometry"));
  this->declare_parameter<double>("wheel_base", 2.875);

  std::string vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
  std::string odometry_output_topic_ =
    this->get_parameter("wheel_odometry_output_topic").as_string();
  double wheel_base_ = this->get_parameter("wheel_base").as_double();

  // Update wheel base size based on parameter
  wheel_odometry_.setWheelBase(wheel_base_);

  // Subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // keep only last 10 messages
  vehicle_status_sub_ = this->create_subscription<interfacing_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, qos,
    std::bind(&WheelOdometryNode::vehicleStatusCallback, this, std::placeholders::_1));

  // Publisher
  wheel_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    odometry_output_topic_, qos);
}

void WheelOdometryNode::vehicleStatusCallback(
  const interfacing_msgs::msg::VehicleStatus::SharedPtr msg)
{
  // Use core logic to compute odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  wheel_odometry_.computeOdometry(msg, odom_msg);

  // Publish odometry message
  wheel_odometry_publisher_->publish(odom_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
