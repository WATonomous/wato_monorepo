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

#pragma once

#include <Eigen/Core>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "patchworkpp/ground_removal_core.hpp"

namespace wato::percpetion::patchworkpp
{

class GroundRemovalNode : public rclcpp::Node
{
public:
  GroundRemovalNode() = delete;
  explicit GroundRemovalNode(const rclcpp::NodeOptions & options);

  static constexpr auto kCloudTopic = "input_cloud";
  static constexpr auto kGroundTopic = "ground_cloud";
  static constexpr auto kNonGroundTopic = "non_ground_cloud";

private:
  void declareParameters(patchwork::Params & params);
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void publishSegments(
    const Eigen::MatrixX3f & ground_points,
    const Eigen::MatrixX3f & nonground_points,
    const std_msgs::msg::Header & header);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  std::unique_ptr<GroundRemovalCore> core_;
};

}  // namespace wato::percpetion::patchworkpp
