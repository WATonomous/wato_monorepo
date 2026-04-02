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

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eidos/plugins/base_visualization_plugin.hpp"

namespace eidos
{

/**
 * @brief Publishes factor graph state markers for RViz.
 *
 * Renders optimized poses as spheres and factor connections as lines.
 */
class FactorGraphVisualization : public VisualizationPlugin
{
public:
  FactorGraphVisualization() = default;
  ~FactorGraphVisualization() override = default;

  /// @brief Declare ROS parameters (scale, line width, mode) and create the marker publisher.
  void onInitialize() override;

protected:
  /// @brief Reset publish timer state on activation.
  void onActivate() override;

  /// @brief Clean up on deactivation.
  void onDeactivate() override;

  /**
   * @brief Render factor graph state as RViz markers.
   *
   * Publishes optimized poses as sphere markers and factor connections
   * (between-factors, loop closures) as line markers. Supports "full" and
   * "windowed" modes.
   *
   * @param optimized_values Latest optimized GTSAM Values for pose lookups.
   */
  void render(const gtsam::Values & optimized_values) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  std::string map_frame_;
  double state_scale_;
  double line_width_;
  double publish_rate_ = 1.0;
  std::string mode_ = "full";
  double window_radius_ = 50.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace eidos
