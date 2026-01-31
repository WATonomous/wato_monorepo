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

#ifndef lanelet_markers__LANE_CONTEXT_MARKERS_NODE_HPP_
#define lanelet_markers__LANE_CONTEXT_MARKERS_NODE_HPP_

#include <string>

#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lanelet_markers
{

/**
 * @brief Converts CurrentLaneContext messages to visualization markers.
 */
class LaneContextMarkersNode : public rclcpp::Node
{
public:
  explicit LaneContextMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for incoming CurrentLaneContext messages.
   *
   * Clears previous markers, then creates lane boundary, centerline,
   * direction arrow, info text, event distance, stop line, and
   * traffic light markers for the current lanelet context.
   *
   * @param msg Current lane context with lanelet geometry and event distances.
   */
  void laneContextCallback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);

  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  double boundary_line_width_;
  double centerline_line_width_;
};

}  // namespace lanelet_markers

#endif  // lanelet_markers__LANE_CONTEXT_MARKERS_NODE_HPP_
