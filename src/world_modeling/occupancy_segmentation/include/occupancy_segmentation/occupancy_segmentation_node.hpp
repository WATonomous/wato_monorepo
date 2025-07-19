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

#ifndef OCCUPANCY_SEGMENTATION_NODE_HPP_
#define OCCUPANCY_SEGMENTATION_NODE_HPP_
#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "occupancy_segmentation/occupancy_segmentation_core.hpp"
#include "rclcpp/rclcpp.hpp"

typedef std::chrono::high_resolution_clock Clock;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,  // here we assume a XYZ + "test" (as fields)
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(u_int16_t, ring, ring)(float, time, time))

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 *
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects  messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class OccupancySegmentationNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Transformer node constructor.
   */
  OccupancySegmentationNode();

private:
  // Object that handles data processing and validation.
  OccupancySegmentationCore<PointXYZIRT> _patchwork;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _nonground_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _ground_publisher;

  void subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud);
};

#endif  // OCCUPANCY_SEGMENTATION_NODE_HPP_
