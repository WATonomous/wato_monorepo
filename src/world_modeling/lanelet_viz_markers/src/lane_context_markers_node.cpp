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

#include "lanelet_viz_markers/lane_context_markers_node.hpp"

#include <memory>
#include <string>

#include "lanelet_viz_markers/marker_utils.hpp"

namespace lanelet_viz_markers
{

LaneContextMarkersNode::LaneContextMarkersNode(const rclcpp::NodeOptions & options)
: Node("lane_context_markers_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "/lane_context");
  this->declare_parameter<std::string>("output_topic", "/lane_context_markers");
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("boundary_line_width", 0.25);
  this->declare_parameter<double>("centerline_line_width", 0.1);

  // Get parameters
  auto input_topic = this->get_parameter("input_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  boundary_line_width_ = this->get_parameter("boundary_line_width").as_double();
  centerline_line_width_ = this->get_parameter("centerline_line_width").as_double();

  // Create publisher
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);

  // Create subscription
  subscription_ = this->create_subscription<lanelet_msgs::msg::CurrentLaneContext>(
    input_topic, 10, std::bind(&LaneContextMarkersNode::laneContextCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "LaneContextMarkersNode started: subscribing to '%s', publishing to '%s'",
    input_topic.c_str(),
    output_topic.c_str());
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
  auto delete_left = createDeleteAllMarker("current_left_boundary", frame_id_);
  delete_left.header.stamp = stamp;
  marker_array.markers.push_back(delete_left);

  auto delete_right = createDeleteAllMarker("current_right_boundary", frame_id_);
  delete_right.header.stamp = stamp;
  marker_array.markers.push_back(delete_right);

  auto delete_center = createDeleteAllMarker("current_centerline", frame_id_);
  delete_center.header.stamp = stamp;
  marker_array.markers.push_back(delete_center);

  const auto & lanelet = msg->current_lanelet;

  // Bright green for current lanelet (highlighted)
  auto highlight_color = makeColor(0.0f, 1.0f, 0.4f);

  // Left boundary
  if (!lanelet.left_boundary.empty()) {
    auto left_marker = createLineStripMarker(
      "current_left_boundary", marker_id++, frame_id_, lanelet.left_boundary, highlight_color, boundary_line_width_);
    left_marker.header.stamp = stamp;
    marker_array.markers.push_back(left_marker);
  }

  // Right boundary
  if (!lanelet.right_boundary.empty()) {
    auto right_marker = createLineStripMarker(
      "current_right_boundary", marker_id++, frame_id_, lanelet.right_boundary, highlight_color, boundary_line_width_);
    right_marker.header.stamp = stamp;
    marker_array.markers.push_back(right_marker);
  }

  // Centerline
  if (!lanelet.centerline.empty()) {
    auto centerline_color = makeColor(0.0f, 0.8f, 0.3f, 0.8f);  // Slightly different green
    auto centerline_marker = createLineStripMarker(
      "current_centerline", marker_id++, frame_id_, lanelet.centerline, centerline_color, centerline_line_width_);
    centerline_marker.header.stamp = stamp;
    marker_array.markers.push_back(centerline_marker);
  }

  publisher_->publish(marker_array);
}

}  // namespace lanelet_viz_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet_viz_markers::LaneContextMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
