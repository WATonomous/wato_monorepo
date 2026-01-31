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

#ifndef lanelet_markers__MAP_VIZ_MARKERS_NODE_HPP_
#define lanelet_markers__MAP_VIZ_MARKERS_NODE_HPP_

#include <string>

#include "lanelet_msgs/msg/map_visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lanelet_markers
{

/**
 * @brief Converts MapVisualization messages to visualization markers.
 */
class MapVizMarkersNode : public rclcpp::Node
{
public:
  explicit MapVizMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for incoming MapVisualization messages.
   *
   * Clears previous markers, then creates lane boundary, centerline,
   * direction arrow, stop line, traffic light, speed limit, and yield
   * sign markers for all lanelets in the visualization message.
   *
   * @param msg Map visualization data containing lanelets with geometry and regulatory elements.
   */
  void mapVisualizationCallback(const lanelet_msgs::msg::MapVisualization::SharedPtr msg);

  rclcpp::Subscription<lanelet_msgs::msg::MapVisualization>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  bool show_centerlines_;
  bool show_stop_lines_;
  bool show_traffic_lights_;
  bool show_direction_arrows_;
  bool show_speed_limits_;
  bool show_yield_markers_;
  bool use_boundary_colors_;
  double boundary_line_width_;
  double centerline_line_width_;
  double stop_line_width_;
  double traffic_light_radius_;
  double direction_arrow_spacing_;
  double direction_arrow_size_;
  double speed_limit_text_height_;
  double yield_marker_size_;
};

}  // namespace lanelet_markers

#endif  // lanelet_markers__MAP_VIZ_MARKERS_NODE_HPP_
