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

#ifndef COSTMAP_MARKERS__COSTMAP_MARKERS_NODE_HPP_
#define COSTMAP_MARKERS__COSTMAP_MARKERS_NODE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace costmap_markers
{

/**
 * @brief Converts a PolygonStamped footprint into a LINE_STRIP MarkerArray.
 *
 * Subscribes to a footprint polygon (published by costmap_node) and
 * re-publishes it as a visualization_msgs MarkerArray for rviz2 / Foxglove.
 * Color and line width are configurable via ROS parameters.
 */
class CostmapMarkersNode : public rclcpp::Node
{
public:
  CostmapMarkersNode();

private:
  /** @brief Build a closed LINE_STRIP marker from the polygon and publish. */
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  std::vector<double> footprint_color_;
  double footprint_line_width_;
};

}  // namespace costmap_markers

#endif  // COSTMAP_MARKERS__COSTMAP_MARKERS_NODE_HPP_
