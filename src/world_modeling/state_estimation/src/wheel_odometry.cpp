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

#include "state_estimation/wheel_odometry.hpp"

#include <string>

WheelOdometry::WheelOdometry()
: Node("wheel_odometry")
{
  // Input and Output Topic Names
  this->declare_parameter<std::string>("vehicle_status_topic", std::string("/vehicle_status"));
  this->declare_parameter<std::string>("wheel_odometry_output_topic", std::string("/state_estimation/wheel_odometry"));

  // Other Parameters
  this->declare_parameter<double>("wheel_base", 2.875);

  std::string vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
  std::string odometry_output_topic_ = this->get_parameter("wheel_odometry_output_topic").as_string();
  wheel_base_ = this->get_parameter("wheel_base").as_double();

  // Subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // keep only last 10 messages
  vehicle_status_sub_ = this->create_subscription<interfacing_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, qos, std::bind(&WheelOdometry::vehicleStatusCallback, this, std::placeholders::_1));

  // Publisher
  wheel_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_output_topic_, qos);
}

void WheelOdometry::vehicleStatusCallback(const interfacing_msgs::msg::VehicleStatus::SharedPtr msg)
{
  // Get current vehicle speed and steering angle
  double speed = msg->speed;  // in m/s
  double steering_angle = msg->steering_angle;  // in radians

  // Update bicycle model based on received data
  // Note: The twist part of the odometry message is in the child frame (base_link)
  double _forward_velocity = speed;  // vehicle moves forward at the speed
  double _lateral_velocity = 0.0;  // vehicle cannot move laterally
  double _angular_velocity = -(speed / wheel_base_) * tan(steering_angle);

  // Create odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = _forward_velocity;
  odom_msg.twist.twist.linear.y = _lateral_velocity;
  odom_msg.twist.twist.angular.z = _angular_velocity;

  // Publish odometry message
  wheel_odometry_publisher_->publish(odom_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
