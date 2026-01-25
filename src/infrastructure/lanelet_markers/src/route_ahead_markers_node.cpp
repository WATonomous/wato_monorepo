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

#include "lanelet_markers/route_ahead_markers_node.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"

namespace lanelet_markers
{

RouteAheadMarkersNode::RouteAheadMarkersNode(const rclcpp::NodeOptions & options)
: Node("route_ahead_markers_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("centerline_line_width", 0.4);
  this->declare_parameter<bool>("show_lanelet_ids", false);
  this->declare_parameter<bool>("show_route_info", true);

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  centerline_line_width_ = this->get_parameter("centerline_line_width").as_double();
  show_lanelet_ids_ = this->get_parameter("show_lanelet_ids").as_bool();
  show_route_info_ = this->get_parameter("show_route_info").as_bool();

  // Create publisher and subscription
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<lanelet_msgs::msg::RouteAhead>(
    "route_ahead", 10, std::bind(&RouteAheadMarkersNode::routeAheadCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RouteAheadMarkersNode started");
}

void RouteAheadMarkersNode::routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  // Use message timestamp, fallback to now() if not set
  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {"route_path", "route_info", "route_ids"};
  for (const auto & ns : namespaces) {
    auto delete_marker = createDeleteAllMarker(ns, frame_id_);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  // If no active route, just publish delete markers
  if (!msg->has_active_route || msg->lanelets.empty()) {
    publisher_->publish(marker_array);
    return;
  }

  // Collect all centerline points into one continuous path
  std::vector<geometry_msgs::msg::Point> route_path;
  for (const auto & lanelet : msg->lanelets) {
    for (const auto & pt : lanelet.centerline) {
      route_path.push_back(pt);
    }
  }

  // Route path overlay (bright cyan color)
  if (!route_path.empty()) {
    auto path_color = makeColor(0.0f, 0.8f, 1.0f, 0.7f);
    auto path_marker =
      createLineStripMarker("route_path", marker_id++, frame_id_, route_path, path_color, centerline_line_width_);
    path_marker.header.stamp = stamp;
    marker_array.markers.push_back(path_marker);
  }

  // Route info text at the first lanelet
  if (show_route_info_ && !route_path.empty()) {
    geometry_msgs::msg::Point text_pos = route_path.front();
    text_pos.z += 2.5;

    std::ostringstream oss;
    oss << "Route: " << msg->lanelets.size() << " lanelets | " << std::fixed << std::setprecision(0)
        << msg->total_distance_m << "m";

    auto info_color = makeColor(0.0f, 0.9f, 1.0f, 0.95f);
    auto info_marker = createTextMarker("route_info", marker_id++, frame_id_, text_pos, oss.str(), info_color, 0.8);
    info_marker.header.stamp = stamp;
    marker_array.markers.push_back(info_marker);
  }

  // Lanelet IDs (optional, off by default)
  if (show_lanelet_ids_) {
    for (const auto & lanelet : msg->lanelets) {
      if (lanelet.centerline.size() >= 2) {
        size_t mid = lanelet.centerline.size() / 2;
        geometry_msgs::msg::Point text_pos = lanelet.centerline[mid];
        text_pos.z += 1.0;

        auto id_color = makeColor(0.7f, 0.9f, 1.0f, 0.8f);
        auto id_marker =
          createTextMarker("route_ids", marker_id++, frame_id_, text_pos, std::to_string(lanelet.id), id_color, 0.5);
        id_marker.header.stamp = stamp;
        marker_array.markers.push_back(id_marker);
      }
    }
  }

  publisher_->publish(marker_array);
}

}  // namespace lanelet_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet_markers::RouteAheadMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
