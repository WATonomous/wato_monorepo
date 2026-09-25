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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <pcl/point_cloud.h>

#include <optional>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Publishes the keyframe cloud pair for the most recent loop closure.
 *
 * Shows one closure at a time rather than accumulating, so the query scan and its
 * matched historical scan stay legible. Optionally dumps each closure (both clouds
 * plus the pre-GICP and post-GICP relative transforms) to disk for offline figures.
 */
class LoopClosureCloudVisualization : public VisualizationPlugin
{
public:
  LoopClosureCloudVisualization() = default;
  ~LoopClosureCloudVisualization() override = default;

  /// @brief Declare ROS parameters and create the cloud/marker publishers.
  void onInitialize() override;

protected:
  /// @brief Reset which closures have been seen.
  void onActivate() override;

  /// @brief Clear state on deactivation.
  void onDeactivate() override;

  /**
   * @brief Publish the most recent loop closure's matched cloud pair.
   *
   * @param optimized_values Latest optimized GTSAM Values (used for pose lookups).
   */
  void render(const gtsam::Values & optimized_values) override;

private:
  /// @brief Write one closure's clouds and transforms to dump_dir_ as a .npz-style text bundle.
  void dumpClosure(
    gtsam::Key source_key,
    gtsam::Key target_key,
    const pcl::PointCloud<PointType> & source_body,
    const pcl::PointCloud<PointType> & target_body,
    const gtsam::Pose3 & initial,
    const gtsam::Pose3 & corrected) const;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string loop_closure_factor_name_;
  std::string loop_target_key_;
  std::string loop_initial_key_;
  std::string loop_corrected_key_;
  std::string pointcloud_from_;
  std::string map_frame_;
  std::string dump_dir_;
  double publish_rate_ = 1.0;
  double line_width_ = 0.5;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::set<gtsam::Key> dumped_keys_;
  std::optional<gtsam::Key> latest_source_key_;
};

}  // namespace eidos
