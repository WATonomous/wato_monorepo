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

#include "occupancy/occupancy_node.hpp"

#include <memory>
#include <string>

#include <rclcpp/clock.hpp>

OccupancyNode::OccupancyNode()
: Node("occupancy")
{
  // Declare ROS parameters
  this->declare_parameter<std::string>("subscription_topic", std::string("/nonground_points"));
  this->declare_parameter<std::string>("publish_topic", std::string("/costmap"));
  this->declare_parameter<int>("resolution", 3);

  // Fetch parameters
  auto input_topic = this->get_parameter("subscription_topic").as_string();
  auto output_topic = this->get_parameter("publish_topic").as_string();
  auto resolution = this->get_parameter("resolution").as_int();

  occupancy_ = OccupancyCore(resolution);

  _subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, ADVERTISING_FREQ, std::bind(&OccupancyNode::subscription_callback, this, std::placeholders::_1));

  _publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, 10);
}

void OccupancyNode::subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received message: %s", msg->header.frame_id.c_str());

  auto start = rclcpp::Clock().now();
  nav_msgs::msg::OccupancyGrid output_costmap = occupancy_.remove_z_dimension(msg);
  auto end = rclcpp::Clock().now();

  int duration = (end - start).to_chrono<std::chrono::milliseconds>().count();
  RCLCPP_DEBUG(this->get_logger(), "Runtime for dimension reduction: %i ms", duration);
  RCLCPP_DEBUG(this->get_logger(), "3D points: %i", static_cast<int>(msg->width));
  RCLCPP_DEBUG(this->get_logger(), "Height: %i, Width: %i", output_costmap.info.width, output_costmap.info.height);
  RCLCPP_DEBUG(this->get_logger(), "Header outgoing: %s", output_costmap.header.frame_id.c_str());

  _publisher->publish(output_costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
