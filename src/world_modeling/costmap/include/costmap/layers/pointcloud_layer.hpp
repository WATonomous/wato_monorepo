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

#ifndef COSTMAP__LAYERS__POINTCLOUD_LAYER_HPP_
#define COSTMAP__LAYERS__POINTCLOUD_LAYER_HPP_

#include <mutex>
#include <string>

#include "costmap/costmap_layer.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace costmap
{

class PointCloudLayer : public CostmapLayer
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & layer_name,
    tf2_ros::Buffer * tf_buffer) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void update(
    nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::TransformStamped & map_to_costmap) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  tf2_ros::Buffer * tf_buffer_{nullptr};
  std::string layer_name_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::mutex data_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  double max_height_m_{0.0};
  double min_height_m_{0.0};
};

}  // namespace costmap

#endif  // COSTMAP__LAYERS__POINTCLOUD_LAYER_HPP_
