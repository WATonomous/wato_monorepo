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

#ifndef lanelet_markers__AREA_OCCUPANCY_MARKERS_NODE_HPP_
#define lanelet_markers__AREA_OCCUPANCY_MARKERS_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "world_model_msgs/msg/area_occupancy.hpp"

namespace lanelet_markers
{

/**
 * @brief Converts AreaOccupancy messages to visualization markers.
 */
class AreaOccupancyMarkersNode : public rclcpp::Node
{
public:
  explicit AreaOccupancyMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for incoming AreaOccupancy messages.
   *
   * Clears previous markers, then creates boundary outlines, text labels,
   * and object sphere markers for each area in the message.
   *
   * @param msg Area occupancy data containing area definitions and their objects.
   */
  void areaOccupancyCallback(const world_model_msgs::msg::AreaOccupancy::SharedPtr msg);

  rclcpp::Subscription<world_model_msgs::msg::AreaOccupancy>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  double boundary_line_width_;
  double object_marker_radius_;
  double label_text_height_;
};

}  // namespace lanelet_markers

#endif  // lanelet_markers__AREA_OCCUPANCY_MARKERS_NODE_HPP_
