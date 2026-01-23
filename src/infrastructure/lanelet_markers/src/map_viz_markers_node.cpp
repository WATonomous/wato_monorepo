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

#include "lanelet_markers/map_viz_markers_node.hpp"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_markers/marker_utils.hpp"

namespace lanelet_markers
{

MapVizMarkersNode::MapVizMarkersNode(const rclcpp::NodeOptions & options)
: Node("map_viz_markers_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "/map_visualization");
  this->declare_parameter<std::string>("output_topic", "/map_viz_markers");
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<bool>("show_centerlines", true);
  this->declare_parameter<bool>("show_stop_lines", true);
  this->declare_parameter<bool>("show_traffic_lights", true);
  this->declare_parameter<bool>("show_direction_arrows", true);
  this->declare_parameter<bool>("show_speed_limits", false);
  this->declare_parameter<bool>("show_yield_markers", true);
  this->declare_parameter<bool>("use_boundary_colors", true);
  this->declare_parameter<double>("boundary_line_width", 0.15);
  this->declare_parameter<double>("centerline_line_width", 0.05);
  this->declare_parameter<double>("stop_line_width", 0.3);
  this->declare_parameter<double>("traffic_light_radius", 0.5);
  this->declare_parameter<double>("direction_arrow_spacing", 15.0);
  this->declare_parameter<double>("direction_arrow_size", 0.8);
  this->declare_parameter<double>("speed_limit_text_height", 1.0);
  this->declare_parameter<double>("yield_marker_size", 1.5);

  // Get parameters
  auto input_topic = this->get_parameter("input_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  show_centerlines_ = this->get_parameter("show_centerlines").as_bool();
  show_stop_lines_ = this->get_parameter("show_stop_lines").as_bool();
  show_traffic_lights_ = this->get_parameter("show_traffic_lights").as_bool();
  show_direction_arrows_ = this->get_parameter("show_direction_arrows").as_bool();
  show_speed_limits_ = this->get_parameter("show_speed_limits").as_bool();
  show_yield_markers_ = this->get_parameter("show_yield_markers").as_bool();
  use_boundary_colors_ = this->get_parameter("use_boundary_colors").as_bool();
  boundary_line_width_ = this->get_parameter("boundary_line_width").as_double();
  centerline_line_width_ = this->get_parameter("centerline_line_width").as_double();
  stop_line_width_ = this->get_parameter("stop_line_width").as_double();
  traffic_light_radius_ = this->get_parameter("traffic_light_radius").as_double();
  direction_arrow_spacing_ = this->get_parameter("direction_arrow_spacing").as_double();
  direction_arrow_size_ = this->get_parameter("direction_arrow_size").as_double();
  speed_limit_text_height_ = this->get_parameter("speed_limit_text_height").as_double();
  yield_marker_size_ = this->get_parameter("yield_marker_size").as_double();

  // Create publisher
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);

  // Create subscription
  subscription_ = this->create_subscription<lanelet_msgs::msg::MapVisualization>(
    input_topic, 10, std::bind(&MapVizMarkersNode::mapVisualizationCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "MapVizMarkersNode started: subscribing to '%s', publishing to '%s'",
    input_topic.c_str(),
    output_topic.c_str());
}

void MapVizMarkersNode::mapVisualizationCallback(const lanelet_msgs::msg::MapVisualization::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int32_t marker_id = 0;

  // Use message timestamp, fallback to now() if not set
  auto stamp = msg->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = this->get_clock()->now();
  }

  // Delete all previous markers
  auto delete_left = createDeleteAllMarker("left_boundaries", frame_id_);
  delete_left.header.stamp = stamp;
  marker_array.markers.push_back(delete_left);

  auto delete_right = createDeleteAllMarker("right_boundaries", frame_id_);
  delete_right.header.stamp = stamp;
  marker_array.markers.push_back(delete_right);

  if (show_centerlines_) {
    auto delete_center = createDeleteAllMarker("centerlines", frame_id_);
    delete_center.header.stamp = stamp;
    marker_array.markers.push_back(delete_center);
  }
  if (show_stop_lines_) {
    auto delete_stop = createDeleteAllMarker("stop_lines", frame_id_);
    delete_stop.header.stamp = stamp;
    marker_array.markers.push_back(delete_stop);
  }
  if (show_traffic_lights_) {
    auto delete_tl = createDeleteAllMarker("traffic_lights", frame_id_);
    delete_tl.header.stamp = stamp;
    marker_array.markers.push_back(delete_tl);
  }
  if (show_direction_arrows_) {
    auto delete_arrows = createDeleteAllMarker("direction_arrows", frame_id_);
    delete_arrows.header.stamp = stamp;
    marker_array.markers.push_back(delete_arrows);
  }
  if (show_speed_limits_) {
    auto delete_speed = createDeleteAllMarker("speed_limits", frame_id_);
    delete_speed.header.stamp = stamp;
    marker_array.markers.push_back(delete_speed);
  }
  if (show_yield_markers_) {
    auto delete_yield = createDeleteAllMarker("yield_signs", frame_id_);
    delete_yield.header.stamp = stamp;
    marker_array.markers.push_back(delete_yield);
  }

  // Process each lanelet
  for (const auto & lanelet : msg->lanelets) {
    auto base_color = getColorForLaneletType(lanelet.lanelet_type);

    // Left boundary - use boundary type and color if enabled
    if (!lanelet.left_boundary.empty()) {
      auto left_color = use_boundary_colors_
                          ? getColorForBoundary(lanelet.left_boundary_type, lanelet.left_boundary_color)
                          : base_color;

      // Use dashed line for dashed boundaries or where lane change is allowed
      bool is_dashed =
        (lanelet.left_boundary_type == lanelet_msgs::msg::Lanelet::BOUNDARY_DASHED) || lanelet.can_change_left;

      if (is_dashed) {
        auto left_marker = createDashedLineMarker(
          "left_boundaries", marker_id++, frame_id_, lanelet.left_boundary, left_color, boundary_line_width_, 1.5, 1.5);
        left_marker.header.stamp = stamp;
        marker_array.markers.push_back(left_marker);
      } else {
        auto left_marker = createLineStripMarker(
          "left_boundaries", marker_id++, frame_id_, lanelet.left_boundary, left_color, boundary_line_width_);
        left_marker.header.stamp = stamp;
        marker_array.markers.push_back(left_marker);
      }
    }

    // Right boundary - use boundary type and color if enabled
    if (!lanelet.right_boundary.empty()) {
      auto right_color = use_boundary_colors_
                           ? getColorForBoundary(lanelet.right_boundary_type, lanelet.right_boundary_color)
                           : base_color;

      // Use dashed line for dashed boundaries or where lane change is allowed
      bool is_dashed =
        (lanelet.right_boundary_type == lanelet_msgs::msg::Lanelet::BOUNDARY_DASHED) || lanelet.can_change_right;

      if (is_dashed) {
        auto right_marker = createDashedLineMarker(
          "right_boundaries",
          marker_id++,
          frame_id_,
          lanelet.right_boundary,
          right_color,
          boundary_line_width_,
          1.5,
          1.5);
        right_marker.header.stamp = stamp;
        marker_array.markers.push_back(right_marker);
      } else {
        auto right_marker = createLineStripMarker(
          "right_boundaries", marker_id++, frame_id_, lanelet.right_boundary, right_color, boundary_line_width_);
        right_marker.header.stamp = stamp;
        marker_array.markers.push_back(right_marker);
      }
    }

    // Centerline (optional)
    if (show_centerlines_ && !lanelet.centerline.empty()) {
      auto centerline_color = base_color;
      centerline_color.a = 0.5f;  // Semi-transparent
      auto centerline_marker = createLineStripMarker(
        "centerlines", marker_id++, frame_id_, lanelet.centerline, centerline_color, centerline_line_width_);
      centerline_marker.header.stamp = stamp;
      marker_array.markers.push_back(centerline_marker);
    }

    // Direction arrows along centerline
    if (show_direction_arrows_ && !lanelet.centerline.empty() && lanelet.centerline.size() >= 2) {
      double accumulated = 0.0;
      auto arrow_color = makeColor(0.3f, 0.8f, 0.3f, 0.8f);  // Green

      for (size_t i = 1; i < lanelet.centerline.size(); ++i) {
        double dx = lanelet.centerline[i].x - lanelet.centerline[i - 1].x;
        double dy = lanelet.centerline[i].y - lanelet.centerline[i - 1].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        accumulated += dist;

        if (accumulated >= direction_arrow_spacing_) {
          // Create arrow pointing in lane direction
          geometry_msgs::msg::Point arrow_start = lanelet.centerline[i - 1];
          geometry_msgs::msg::Point arrow_end = lanelet.centerline[i];

          // Scale arrow to fixed size
          double len = std::sqrt(dx * dx + dy * dy);
          if (len > 0.1) {
            double scale = direction_arrow_size_ / len;
            arrow_end.x = arrow_start.x + dx * scale;
            arrow_end.y = arrow_start.y + dy * scale;
            arrow_end.z = arrow_start.z + 0.3;  // Lift slightly
            arrow_start.z += 0.3;

            auto arrow = createArrowMarker(
              "direction_arrows", marker_id++, frame_id_, arrow_start, arrow_end, arrow_color, 0.1, 0.25, 0.3);
            arrow.header.stamp = stamp;
            marker_array.markers.push_back(arrow);
          }
          accumulated = 0.0;
        }
      }
    }

    // Speed limit labels
    if (show_speed_limits_ && lanelet.speed_limit_mps > 0 && !lanelet.centerline.empty()) {
      size_t mid = lanelet.centerline.size() / 2;
      double speed_kph = lanelet.speed_limit_mps * 3.6;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(0) << speed_kph << " km/h";

      auto text_color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
      auto text_marker = createTextMarker(
        "speed_limits",
        marker_id++,
        frame_id_,
        lanelet.centerline[mid],
        oss.str(),
        text_color,
        speed_limit_text_height_);
      text_marker.header.stamp = stamp;
      marker_array.markers.push_back(text_marker);
    }

    // Yield sign markers
    if (show_yield_markers_ && lanelet.has_yield_sign && !lanelet.centerline.empty()) {
      auto yield_color = makeColor(1.0f, 0.6f, 0.0f, 0.9f);  // Orange
      auto yield_marker = createTriangleMarker(
        "yield_signs", marker_id++, frame_id_, lanelet.centerline.front(), yield_color, yield_marker_size_);
      yield_marker.header.stamp = stamp;
      marker_array.markers.push_back(yield_marker);
    }

    // Stop lines
    if (show_stop_lines_) {
      for (const auto & stop_line : lanelet.stop_lines) {
        if (!stop_line.points.empty()) {
          auto stop_line_color = makeColor(1.0f, 0.0f, 0.0f);  // Red
          auto stop_marker = createLineStripMarker(
            "stop_lines", marker_id++, frame_id_, stop_line.points, stop_line_color, stop_line_width_);
          stop_marker.header.stamp = stamp;
          marker_array.markers.push_back(stop_marker);
        }
      }
    }

    // Traffic lights
    if (show_traffic_lights_) {
      for (const auto & traffic_light : lanelet.traffic_lights) {
        auto tl_color = makeColor(1.0f, 1.0f, 0.0f);  // Yellow
        auto tl_marker = createSphereMarker(
          "traffic_lights", marker_id++, frame_id_, traffic_light.position, tl_color, traffic_light_radius_);
        tl_marker.header.stamp = stamp;
        marker_array.markers.push_back(tl_marker);
      }
    }
  }

  publisher_->publish(marker_array);
}

}  // namespace lanelet_markers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet_markers::MapVizMarkersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
