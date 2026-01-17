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

#include "vision_msgs_markers/vision_msgs_markers_node.hpp"

#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace vision_msgs_markers
{

VisionMsgsMarkersNode::VisionMsgsMarkersNode()
: Node("vision_msgs_markers")
{
  this->declare_parameter("marker_alpha", 1.0);
  this->declare_parameter("fill_alpha", 0.3);
  this->declare_parameter("text_scale", 0.3);
  this->declare_parameter("line_width", 0.1);

  marker_alpha_ = this->get_parameter("marker_alpha").as_double();
  fill_alpha_ = this->get_parameter("fill_alpha").as_double();
  text_scale_ = this->get_parameter("text_scale").as_double();
  line_width_ = this->get_parameter("line_width").as_double();

  // Distinct color palette for different classes (RGB)
  color_palette_ = {
    {1.0f, 0.0f, 0.0f},  // Red
    {0.0f, 1.0f, 0.0f},  // Green
    {0.0f, 0.0f, 1.0f},  // Blue
    {1.0f, 1.0f, 0.0f},  // Yellow
    {1.0f, 0.0f, 1.0f},  // Magenta
    {0.0f, 1.0f, 1.0f},  // Cyan
    {1.0f, 0.5f, 0.0f},  // Orange
    {0.5f, 0.0f, 1.0f},  // Purple
    {0.0f, 1.0f, 0.5f},  // Spring Green
    {1.0f, 0.0f, 0.5f},  // Rose
    {0.5f, 1.0f, 0.0f},  // Lime
    {0.0f, 0.5f, 1.0f},  // Sky Blue
  };

  detections_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    kInputTopic, 10, std::bind(&VisionMsgsMarkersNode::detectionsCallback, this, std::placeholders::_1));

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kOutputTopic, 10);

  RCLCPP_INFO(this->get_logger(), "VisionMsgsMarkersNode initialized");
}

void VisionMsgsMarkersNode::detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;
  for (const auto & detection : msg->detections) {
    // Create solid fill marker
    auto fill_marker = createFillMarker(detection, msg->header, marker_id++);
    marker_array.markers.push_back(fill_marker);

    // Create box outline marker
    auto box_marker = createBoxMarker(detection, msg->header, marker_id++);
    marker_array.markers.push_back(box_marker);

    // Create text marker above the box
    auto text_marker = createTextMarker(detection, msg->header, marker_id++);
    marker_array.markers.push_back(text_marker);
  }

  markers_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker VisionMsgsMarkersNode::createBoxMarker(
  const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detection_boxes";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Get class ID for coloring
  std::string class_id = "";
  if (!detection.results.empty()) {
    class_id = detection.results[0].hypothesis.class_id;
  }
  auto color = getColorForClassId(class_id);

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = static_cast<float>(marker_alpha_);

  marker.scale.x = line_width_;

  // Get bounding box dimensions and pose
  const auto & bbox = detection.bbox;
  double sx = bbox.size.x / 2.0;
  double sy = bbox.size.y / 2.0;
  double sz = bbox.size.z / 2.0;

  marker.pose = bbox.center;

  // Define 8 corners of the bounding box (in local frame)
  std::array<geometry_msgs::msg::Point, 8> corners;
  // Bottom face
  corners[0].x = -sx;
  corners[0].y = -sy;
  corners[0].z = -sz;
  corners[1].x = sx;
  corners[1].y = -sy;
  corners[1].z = -sz;
  corners[2].x = sx;
  corners[2].y = sy;
  corners[2].z = -sz;
  corners[3].x = -sx;
  corners[3].y = sy;
  corners[3].z = -sz;
  // Top face
  corners[4].x = -sx;
  corners[4].y = -sy;
  corners[4].z = sz;
  corners[5].x = sx;
  corners[5].y = -sy;
  corners[5].z = sz;
  corners[6].x = sx;
  corners[6].y = sy;
  corners[6].z = sz;
  corners[7].x = -sx;
  corners[7].y = sy;
  corners[7].z = sz;

  // 12 edges of the box (pairs of points)
  // Bottom face edges
  marker.points.push_back(corners[0]);
  marker.points.push_back(corners[1]);
  marker.points.push_back(corners[1]);
  marker.points.push_back(corners[2]);
  marker.points.push_back(corners[2]);
  marker.points.push_back(corners[3]);
  marker.points.push_back(corners[3]);
  marker.points.push_back(corners[0]);
  // Top face edges
  marker.points.push_back(corners[4]);
  marker.points.push_back(corners[5]);
  marker.points.push_back(corners[5]);
  marker.points.push_back(corners[6]);
  marker.points.push_back(corners[6]);
  marker.points.push_back(corners[7]);
  marker.points.push_back(corners[7]);
  marker.points.push_back(corners[4]);
  // Vertical edges
  marker.points.push_back(corners[0]);
  marker.points.push_back(corners[4]);
  marker.points.push_back(corners[1]);
  marker.points.push_back(corners[5]);
  marker.points.push_back(corners[2]);
  marker.points.push_back(corners[6]);
  marker.points.push_back(corners[3]);
  marker.points.push_back(corners[7]);

  return marker;
}

visualization_msgs::msg::Marker VisionMsgsMarkersNode::createFillMarker(
  const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detection_fill";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Get class ID for coloring
  std::string class_id = "";
  if (!detection.results.empty()) {
    class_id = detection.results[0].hypothesis.class_id;
  }
  auto color = getColorForClassId(class_id);

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = static_cast<float>(fill_alpha_);

  marker.pose = detection.bbox.center;
  marker.scale.x = detection.bbox.size.x;
  marker.scale.y = detection.bbox.size.y;
  marker.scale.z = detection.bbox.size.z;

  return marker;
}

visualization_msgs::msg::Marker VisionMsgsMarkersNode::createTextMarker(
  const vision_msgs::msg::Detection3D & detection, const std_msgs::msg::Header & header, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detection_labels";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  std::string class_id = "unknown";
  double confidence = 0.0;

  if (!detection.results.empty()) {
    class_id = detection.results[0].hypothesis.class_id;
    confidence = detection.results[0].hypothesis.score;
  }

  auto color = getColorForClassId(class_id);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0;

  marker.scale.z = text_scale_;

  // Position text above the bounding box
  marker.pose = detection.bbox.center;
  marker.pose.position.z += detection.bbox.size.z / 2.0 + text_scale_;

  std::ostringstream ss;
  ss << class_id << " (" << std::fixed << std::setprecision(2) << confidence << ")";
  marker.text = ss.str();

  return marker;
}

std::array<float, 4> VisionMsgsMarkersNode::getColorForClassId(const std::string & class_id)
{
  // Check cache first
  auto it = color_cache_.find(class_id);
  if (it != color_cache_.end()) {
    return it->second;
  }

  // Hash the class_id to get a consistent color index
  size_t hash = std::hash<std::string>{}(class_id);
  size_t color_index = hash % color_palette_.size();

  std::array<float, 4> color = {
    color_palette_[color_index][0],
    color_palette_[color_index][1],
    color_palette_[color_index][2],
    static_cast<float>(marker_alpha_)};

  color_cache_[class_id] = color;
  return color;
}

}  // namespace vision_msgs_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision_msgs_markers::VisionMsgsMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
