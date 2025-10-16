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

namespace wato::perception::patchworkpp
{

class GroundRemovalNode : public rclcpp::Node
{
public:
  GroundRemovalNode() = delete;
  /*
   * Deleted default constructor to prevent unconfigured instances.
   * Use the NodeOptions-based constructor so parameters, remappings,
   * and QoS settings are properly established via rclcpp.
   */
  explicit GroundRemovalNode(const rclcpp::NodeOptions & options);
  /*
   * Constructs the Patchwork++ ROS 2 node.
   * Declares algorithm parameters, initializes the GroundRemovalCore with
   * those parameters, sets up the point cloud subscriber, and creates
   * publishers for ground and non-ground segments using reliable, transient
   * local QoS. The subscriber binds to removeGround() as the callback.
   */

  static constexpr auto kCloudTopic = "input_cloud";
  static constexpr auto kGroundTopic = "ground_cloud";
  static constexpr auto kNonGroundTopic = "non_ground_cloud";

private:
  void declareParameters(patchwork::Params & params);
  /*
   * Declares and populates PatchWork++ parameters on the node.
   * Uses rclcpp::Node::declare_parameter to expose tunables (e.g., sensor
   * height, seed thresholds, distance thresholds, ranges, and verbosity),
   * writing the resolved values into the provided params struct.
   */
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /*
   * Subscription callback for incoming PointCloud2 messages.
   * Converts the ROS cloud into an Eigen matrix, runs core_->process() to
   * segment ground and non-ground, publishes both segments via
   * publishSegments(), and emits a debug log with counts and timing.
   */
  void publishSegments(
    const Eigen::MatrixX3f & ground_points,
    const Eigen::MatrixX3f & nonground_points,
    const std_msgs::msg::Header & header);
  /*
   * Publishes the ground and non-ground point sets as PointCloud2 messages.
   * Converts the provided matrices to PointCloud2 (XYZ, FLOAT32) while
   * preserving the incoming header (frame, timestamp). If publishers are not
   * ready, a throttled warning is logged and publishing is skipped.
   */

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  std::unique_ptr<GroundRemovalCore> core_;
};

}  // namespace wato::perception::patchworkpp
