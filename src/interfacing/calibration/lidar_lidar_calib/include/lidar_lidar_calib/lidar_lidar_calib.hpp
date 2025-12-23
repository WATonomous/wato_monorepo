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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_lidar_calib
{

class LidarLidarCalibNode : public rclcpp::Node
{
public:
  explicit LidarLidarCalibNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg);

  // Message filters for synchronized pointcloud input
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> source_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> target_sub_;

  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace lidar_lidar_calib
