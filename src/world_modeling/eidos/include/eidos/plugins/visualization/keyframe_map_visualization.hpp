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

#include <gtsam/inference/Key.h>
#include <pcl/point_cloud.h>

#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Publishes accumulated or windowed keyframe point cloud map.
 */
class KeyframeMapVisualization : public VisualizationPlugin
{
public:
  KeyframeMapVisualization() = default;
  ~KeyframeMapVisualization() override = default;

  /// @brief Declare ROS parameters (mode, voxel size, etc.) and create the point cloud publisher.
  void onInitialize() override;

protected:
  /// @brief Reset the accumulated cloud and seen/accepted key sets on activation.
  void onActivate() override;

  /// @brief Clear accumulated visualization state on deactivation.
  void onDeactivate() override;

  /**
   * @brief Render accumulated or windowed keyframe point cloud map.
   *
   * In "accumulated" mode, incrementally adds new keyframe clouds (with optional
   * skip factor) and publishes the full voxel-filtered result. In "windowed" mode,
   * assembles clouds from keyframes within a spatial radius of the latest pose.
   *
   * @param optimized_values Latest optimized GTSAM Values (used for pose lookups in SLAM mode).
   */
  void render(const gtsam::Values & optimized_values) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string pointcloud_from_;
  std::string map_frame_;
  std::string mode_;
  float voxel_leaf_size_;
  double publish_rate_;
  int skip_factor_;
  double window_radius_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  pcl::PointCloud<PointType>::Ptr accumulated_cloud_;
  std::set<gtsam::Key> seen_keys_;  ///< All keys we've evaluated (skip logic)
  std::set<gtsam::Key> accepted_keys_;  ///< Keys whose clouds we actually render
  int skip_counter_ = 0;
};

}  // namespace eidos
