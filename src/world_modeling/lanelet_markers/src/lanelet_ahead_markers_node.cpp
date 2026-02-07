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

#include "lanelet_markers/lanelet_ahead_markers_node.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"

namespace lanelet_markers
{

LaneletAheadMarkersNode::LaneletAheadMarkersNode(const rclcpp::NodeOptions & options)
: Node("lanelet_ahead_markers_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("centerline_line_width", 0.4);
  this->declare_parameter<double>("boundary_line_width", 0.15);
  this->declare_parameter<bool>("show_lanelet_ids", false);
  this->declare_parameter<bool>("show_info", true);
  this->declare_parameter<bool>("show_boundaries", true);

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  centerline_line_width_ = this->get_parameter("centerline_line_width").as_double();
  boundary_line_width_ = this->get_parameter("boundary_line_width").as_double();
  show_lanelet_ids_ = this->get_parameter("show_lanelet_ids").as_bool();
  show_info_ = this->get_parameter("show_info").as_bool();
  show_boundaries_ = this->get_parameter("show_boundaries").as_bool();

  // Create publisher and subscription
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<lanelet_msgs::msg::LaneletAhead>(
    "lanelet_ahead", 10, std::bind(&LaneletAheadMarkersNode::laneletAheadCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LaneletAheadMarkersNode started");
}

void LaneletAheadMarkersNode::laneletAheadCallback(const lanelet_msgs::msg::LaneletAhead::SharedPtr msg)
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
    "lanelet_ahead_path",
    "lanelet_ahead_info",
    "lanelet_ahead_ids",
    "lanelet_ahead_left_boundaries",
    "lanelet_ahead_right_boundaries"};
  for (const auto & ns : namespaces) {
    auto delete_marker = createDeleteAllMarker(ns, frame_id_);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  // If no lanelets, just publish delete markers
  if (msg->lanelets.empty()) {
    publisher_->publish(marker_array);
    return;
  }

  // Create separate markers for each lanelet (centerline and boundaries)
  auto centerline_color = makeColor(0.8f, 0.5f, 1.0f, 0.7f);  // Purple
  auto boundary_color = makeColor(0.8f, 0.5f, 1.0f, 0.5f);  // Purple, slightly transparent
  auto current_color = makeColor(0.2f, 1.0f, 0.4f, 0.9f);  // Green for current lanelet

  for (const auto & lanelet : msg->lanelets) {
    bool is_current = (lanelet.id == msg->current_lanelet_id);
    auto & cl_color = is_current ? current_color : centerline_color;

    // Centerline marker for this lanelet
    if (lanelet.centerline.size() >= 2) {
      auto centerline_marker = createLineStripMarker(
        "lanelet_ahead_path", marker_id++, frame_id_, lanelet.centerline, cl_color, centerline_line_width_);
      centerline_marker.header.stamp = stamp;
      marker_array.markers.push_back(centerline_marker);
    }

    // Dotted boundary markers
    if (show_boundaries_) {
      if (lanelet.left_boundary.size() >= 2) {
        auto left_marker = createDottedLineMarker(
          "lanelet_ahead_left_boundaries",
          marker_id++,
          frame_id_,
          lanelet.left_boundary,
          boundary_color,
          boundary_line_width_ * 2.0,
          1.5);
        left_marker.header.stamp = stamp;
        marker_array.markers.push_back(left_marker);
      }

      if (lanelet.right_boundary.size() >= 2) {
        auto right_marker = createDottedLineMarker(
          "lanelet_ahead_right_boundaries",
          marker_id++,
          frame_id_,
          lanelet.right_boundary,
          boundary_color,
          boundary_line_width_ * 2.0,
          1.5);
        right_marker.header.stamp = stamp;
        marker_array.markers.push_back(right_marker);
      }
    }
  }

  // Get first centerline point for info text position
  geometry_msgs::msg::Point first_point;
  bool has_first_point = false;
  for (const auto & lanelet : msg->lanelets) {
    if (!lanelet.centerline.empty()) {
      first_point = lanelet.centerline.front();
      has_first_point = true;
      break;
    }
  }

  // Info text at the first lanelet
  if (show_info_ && has_first_point) {
    geometry_msgs::msg::Point text_pos = first_point;
    text_pos.z += 2.5;

    std::ostringstream oss;
    oss << "Nearby: " << msg->lanelets.size() << " lanelets | " << std::fixed << std::setprecision(0) << msg->radius_m
        << "m radius";

    auto info_color = makeColor(0.9f, 0.6f, 1.0f, 0.95f);
    auto info_marker =
      createTextMarker("lanelet_ahead_info", marker_id++, frame_id_, text_pos, oss.str(), info_color, 0.8);
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

        auto id_color = makeColor(0.9f, 0.7f, 1.0f, 0.8f);
        auto id_marker = createTextMarker(
          "lanelet_ahead_ids", marker_id++, frame_id_, text_pos, std::to_string(lanelet.id), id_color, 0.5);
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
  auto node = std::make_shared<lanelet_markers::LaneletAheadMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
