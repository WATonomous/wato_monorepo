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

#include "lanelet_markers/area_occupancy_markers_node.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "lanelet_markers/marker_utils.hpp"
#include "world_model_msgs/msg/area_definition.hpp"

namespace lanelet_markers
{

AreaOccupancyMarkersNode::AreaOccupancyMarkersNode(const rclcpp::NodeOptions & options)
: Node("area_occupancy_markers_node", options)
{
  this->declare_parameter<std::string>("frame_id", "");
  this->declare_parameter<double>("boundary_line_width", 0.15);
  this->declare_parameter<double>("object_marker_radius", 0.3);
  this->declare_parameter<double>("label_text_height", 0.6);

  frame_id_ = this->get_parameter("frame_id").as_string();
  boundary_line_width_ = this->get_parameter("boundary_line_width").as_double();
  object_marker_radius_ = this->get_parameter("object_marker_radius").as_double();
  label_text_height_ = this->get_parameter("label_text_height").as_double();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  subscription_ = this->create_subscription<world_model_msgs::msg::AreaOccupancy>(
    "area_occupancy", 10,
    std::bind(&AreaOccupancyMarkersNode::areaOccupancyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "AreaOccupancyMarkersNode started");
}

void AreaOccupancyMarkersNode::areaOccupancyCallback(
  const world_model_msgs::msg::AreaOccupancy::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  // Use parameter frame_id if set, otherwise fall back to message frame_id
  std::string frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  std::vector<std::string> namespaces = {"area_boundaries", "area_labels", "area_objects"};
  for (const auto & ns : namespaces) {
    auto delete_marker = createDeleteAllMarker(ns, frame_id);
    delete_marker.header.stamp = stamp;
    marker_array.markers.push_back(delete_marker);
  }

  for (const auto & area_info : msg->areas) {
    const auto & area = area_info.area;

    // Choose color based on occupancy: green=free, red=occupied
    auto boundary_color = area_info.is_occupied
      ? makeColor(1.0f, 0.0f, 0.0f, 0.6f)
      : makeColor(0.0f, 1.0f, 0.0f, 0.6f);

    // Build boundary points based on area type
    std::vector<geometry_msgs::msg::Point> boundary_points;

    if (area.area_type == world_model_msgs::msg::AreaDefinition::TYPE_RECTANGLE) {
      double half_l = area.length / 2.0;
      double half_w = area.width / 2.0;

      geometry_msgs::msg::Point p;
      p.z = 0.0;

      p.x = area.center_x + half_l; p.y = area.center_y + half_w;
      boundary_points.push_back(p);
      p.x = area.center_x - half_l; p.y = area.center_y + half_w;
      boundary_points.push_back(p);
      p.x = area.center_x - half_l; p.y = area.center_y - half_w;
      boundary_points.push_back(p);
      p.x = area.center_x + half_l; p.y = area.center_y - half_w;
      boundary_points.push_back(p);
      // Close the rectangle
      p.x = area.center_x + half_l; p.y = area.center_y + half_w;
      boundary_points.push_back(p);

    } else if (area.area_type == world_model_msgs::msg::AreaDefinition::TYPE_CIRCLE) {
      const int num_segments = 36;
      for (int i = 0; i <= num_segments; ++i) {
        double angle = 2.0 * M_PI * static_cast<double>(i) / num_segments;
        geometry_msgs::msg::Point p;
        p.x = area.center_x + area.radius * std::cos(angle);
        p.y = area.center_y + area.radius * std::sin(angle);
        p.z = 0.0;
        boundary_points.push_back(p);
      }

    } else if (area.area_type == world_model_msgs::msg::AreaDefinition::TYPE_PARTIAL_CIRCLE) {
      // Arc from start_angle to end_angle
      const int num_segments = 36;

      // Start with center point to draw radial line from center to arc start
      geometry_msgs::msg::Point center;
      center.x = area.center_x;
      center.y = area.center_y;
      center.z = 0.0;
      boundary_points.push_back(center);

      double start = area.start_angle;
      double end = area.end_angle;
      // Handle wrap-around
      if (end < start) {
        end += 2.0 * M_PI;
      }
      double arc_span = end - start;

      for (int i = 0; i <= num_segments; ++i) {
        double angle = start + arc_span * static_cast<double>(i) / num_segments;
        geometry_msgs::msg::Point p;
        p.x = area.center_x + area.radius * std::cos(angle);
        p.y = area.center_y + area.radius * std::sin(angle);
        p.z = 0.0;
        boundary_points.push_back(p);
      }

      // Close back to center
      boundary_points.push_back(center);
    }

    if (!boundary_points.empty()) {
      auto boundary_marker = createLineStripMarker(
        "area_boundaries", marker_id++, frame_id, boundary_points, boundary_color,
        boundary_line_width_);
      boundary_marker.header.stamp = stamp;
      marker_array.markers.push_back(boundary_marker);
    }

    // Text label at area center
    geometry_msgs::msg::Point label_pos;
    label_pos.x = area.center_x;
    label_pos.y = area.center_y;
    label_pos.z = 0.0;

    std::string label_text = area_info.name + "\n" +
      (area_info.is_occupied ? "OCCUPIED" : "FREE");

    auto label_color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    auto label_marker = createTextMarker(
      "area_labels", marker_id++, frame_id, label_pos, label_text, label_color,
      label_text_height_);
    label_marker.header.stamp = stamp;
    marker_array.markers.push_back(label_marker);

    // Sphere marker for each dynamic object in this area
    auto object_color = makeColor(1.0f, 0.6f, 0.0f, 0.8f);
    for (const auto & obj : area_info.objects) {
      auto obj_marker = createSphereMarker(
        "area_objects", marker_id++, frame_id, obj.pose.position, object_color,
        object_marker_radius_);
      obj_marker.header.stamp = stamp;
      marker_array.markers.push_back(obj_marker);
    }
  }

  publisher_->publish(marker_array);
}

}  // namespace lanelet_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet_markers::AreaOccupancyMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
