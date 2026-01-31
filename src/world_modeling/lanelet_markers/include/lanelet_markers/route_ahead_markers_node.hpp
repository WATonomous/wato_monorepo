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

#ifndef lanelet_markers__ROUTE_AHEAD_MARKERS_NODE_HPP_
#define lanelet_markers__ROUTE_AHEAD_MARKERS_NODE_HPP_

#include <string>

#include "lanelet_msgs/msg/route_ahead.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lanelet_markers
{

/**
 * @brief Converts RouteAhead messages to visualization markers.
 */
class RouteAheadMarkersNode : public rclcpp::Node
{
public:
  explicit RouteAheadMarkersNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for incoming RouteAhead messages.
   *
   * Clears previous markers, then creates centerline, boundary,
   * info text, and optional lanelet ID markers for all lanelets
   * along the active route within the lookahead distance.
   *
   * @param msg Route ahead data with lanelets, total distance, and active route flag.
   */
  void routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg);

  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::string frame_id_;
  double centerline_line_width_;
  double boundary_line_width_;
  bool show_lanelet_ids_;
  bool show_route_info_;
  bool show_boundaries_;
};

}  // namespace lanelet_markers

#endif  // lanelet_markers__ROUTE_AHEAD_MARKERS_NODE_HPP_
