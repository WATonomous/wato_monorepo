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

#include "lanelet_markers/lane_context_markers_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace lanelet_markers
{

LaneContextMarkersNode::LaneContextMarkersNode(const rclcpp::NodeOptions & options)
: Node("lane_context_markers_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("boundary_line_width", 0.25);
  this->declare_parameter<double>("centerline_line_width", 0.1);

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  boundary_line_width_ = this->get_parameter("boundary_line_width").as_double();
  centerline_line_width_ = this->get_parameter("centerline_line_width").as_double();

  // Create publisher and subscription (use remaps to override topic names)
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    "lane_context", 10, std::bind(&LaneContextMarkersNode::laneContextCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LaneContextMarkersNode started");
}

void LaneContextMarkersNode::laneContextCallback(const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  // Use message timestamp, fallback to now() if not set
  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {
    "current_left_boundary",
    "current_right_boundary",
    "current_centerline",
    "lane_info",
    "lane_direction",
    "lane_events",
    "stop_lines",
    "traffic_lights"};
  for (const auto & ns : namespaces) {
    auto delete_marker = createDeleteAllMarker(ns, frame_id_);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  const auto & lanelet = msg->current_lanelet;

  // Bright green for current lanelet (highlighted)
  auto highlight_color = makeColor(0.0f, 1.0f, 0.4f);
  auto text_color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);

  // Left boundary with lane change indication
  if (!lanelet.left_boundary.empty()) {
    bool is_dashed =
      lanelet.can_change_left || lanelet.left_boundary_type == lanelet_msgs::msg::Lanelet::BOUNDARY_DASHED;
    if (is_dashed) {
      auto left_marker = createDashedLineMarker(
        "current_left_boundary",
        marker_id++,
        frame_id_,
        lanelet.left_boundary,
        highlight_color,
        boundary_line_width_,
        1.5,
        1.5);
      left_marker.header.stamp = stamp;
      marker_array.markers.push_back(left_marker);
    } else {
      auto left_marker = createLineStripMarker(
        "current_left_boundary", marker_id++, frame_id_, lanelet.left_boundary, highlight_color, boundary_line_width_);
      left_marker.header.stamp = stamp;
      marker_array.markers.push_back(left_marker);
    }
  }

  // Right boundary with lane change indication
  if (!lanelet.right_boundary.empty()) {
    bool is_dashed =
      lanelet.can_change_right || lanelet.right_boundary_type == lanelet_msgs::msg::Lanelet::BOUNDARY_DASHED;
    if (is_dashed) {
      auto right_marker = createDashedLineMarker(
        "current_right_boundary",
        marker_id++,
        frame_id_,
        lanelet.right_boundary,
        highlight_color,
        boundary_line_width_,
        1.5,
        1.5);
      right_marker.header.stamp = stamp;
      marker_array.markers.push_back(right_marker);
    } else {
      auto right_marker = createLineStripMarker(
        "current_right_boundary",
        marker_id++,
        frame_id_,
        lanelet.right_boundary,
        highlight_color,
        boundary_line_width_);
      right_marker.header.stamp = stamp;
      marker_array.markers.push_back(right_marker);
    }
  }

  // Centerline
  if (!lanelet.centerline.empty()) {
    auto centerline_color = makeColor(0.0f, 0.8f, 0.3f, 0.8f);
    auto centerline_marker = createLineStripMarker(
      "current_centerline", marker_id++, frame_id_, lanelet.centerline, centerline_color, centerline_line_width_);
    centerline_marker.header.stamp = stamp;
    marker_array.markers.push_back(centerline_marker);

    // Direction arrow at centerline midpoint
    if (lanelet.centerline.size() >= 2) {
      size_t mid = lanelet.centerline.size() / 2;
      auto arrow_color = makeColor(0.3f, 0.9f, 0.3f, 0.9f);

      geometry_msgs::msg::Point arrow_start = lanelet.centerline[mid];
      geometry_msgs::msg::Point arrow_end = lanelet.centerline[std::min(mid + 1, lanelet.centerline.size() - 1)];

      double dx = arrow_end.x - arrow_start.x;
      double dy = arrow_end.y - arrow_start.y;
      double len = std::sqrt(dx * dx + dy * dy);
      if (len > 0.1) {
        double scale = 2.0 / len;
        arrow_end.x = arrow_start.x + dx * scale;
        arrow_end.y = arrow_start.y + dy * scale;
        arrow_end.z = arrow_start.z + 0.5;
        arrow_start.z += 0.5;

        auto arrow = createArrowMarker(
          "lane_direction", marker_id++, frame_id_, arrow_start, arrow_end, arrow_color, 0.15, 0.35, 0.5);
        arrow.header.stamp = stamp;
        marker_array.markers.push_back(arrow);
      }
    }

    // Lanelet info text (ID, speed limit, type)
    if (!lanelet.centerline.empty()) {
      size_t mid = lanelet.centerline.size() / 2;
      geometry_msgs::msg::Point text_pos = lanelet.centerline[mid];
      text_pos.z += 1.5;

      std::ostringstream oss;
      oss << "ID: " << lanelet.id;
      if (lanelet.speed_limit_mps > 0) {
        oss << " | " << std::fixed << std::setprecision(0) << (lanelet.speed_limit_mps * 3.6) << " km/h";
      }
      if (!lanelet.lanelet_type.empty() && lanelet.lanelet_type != "lanelet") {
        oss << " | " << lanelet.lanelet_type;
      }

      auto info_marker = createTextMarker("lane_info", marker_id++, frame_id_, text_pos, oss.str(), text_color, 0.8);
      info_marker.header.stamp = stamp;
      marker_array.markers.push_back(info_marker);
    }
  }

  // Event indicators
  std::ostringstream events;
  bool has_events = false;

  if (msg->distance_to_lanelet_end_m > 0) {
    events << "End: " << std::fixed << std::setprecision(1) << msg->distance_to_lanelet_end_m << "m";
    has_events = true;
  }
  if (msg->distance_to_traffic_light_m >= 0) {
    if (has_events) events << " | ";
    events << "TL: " << std::fixed << std::setprecision(1) << msg->distance_to_traffic_light_m << "m";
    has_events = true;
  }
  if (msg->distance_to_stop_line_m >= 0) {
    if (has_events) events << " | ";
    events << "Stop: " << std::fixed << std::setprecision(1) << msg->distance_to_stop_line_m << "m";
    has_events = true;
  }
  if (msg->distance_to_intersection_m >= 0) {
    if (has_events) events << " | ";
    events << "Int: " << std::fixed << std::setprecision(1) << msg->distance_to_intersection_m << "m";
    has_events = true;
  }

  if (has_events && !lanelet.centerline.empty()) {
    geometry_msgs::msg::Point event_pos = lanelet.centerline.front();
    event_pos.z += 2.5;

    auto event_color = makeColor(1.0f, 0.9f, 0.2f, 0.9f);
    auto event_marker =
      createTextMarker("lane_events", marker_id++, frame_id_, event_pos, events.str(), event_color, 0.6);
    event_marker.header.stamp = stamp;
    marker_array.markers.push_back(event_marker);
  }

  // Stop lines
  for (const auto & stop_line : lanelet.stop_lines) {
    if (!stop_line.points.empty()) {
      auto stop_color = makeColor(1.0f, 0.0f, 0.0f, 0.9f);
      auto stop_marker = createLineStripMarker("stop_lines", marker_id++, frame_id_, stop_line.points, stop_color, 0.3);
      stop_marker.header.stamp = stamp;
      marker_array.markers.push_back(stop_marker);
    }
  }

  // Traffic lights
  for (const auto & tl : lanelet.traffic_lights) {
    auto tl_color = makeColor(1.0f, 1.0f, 0.0f, 0.9f);
    auto tl_marker = createSphereMarker("traffic_lights", marker_id++, frame_id_, tl.position, tl_color, 0.5);
    tl_marker.header.stamp = stamp;
    marker_array.markers.push_back(tl_marker);
  }

  publisher_->publish(marker_array);
}

}  // namespace lanelet_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet_markers::LaneContextMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
