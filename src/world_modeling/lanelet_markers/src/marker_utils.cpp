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

#include "lanelet_markers/marker_utils.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lanelet_markers
{

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA getColorForLaneletType(const std::string & lanelet_type)
{
  if (lanelet_type == "road") {
    return makeColor(1.0f, 1.0f, 1.0f);  // White
  } else if (lanelet_type == "intersection") {
    return makeColor(1.0f, 0.7f, 0.0f);  // Orange
  } else if (lanelet_type == "crosswalk") {
    return makeColor(0.0f, 1.0f, 1.0f);  // Cyan
  } else if (lanelet_type == "parking") {
    return makeColor(0.3f, 0.3f, 1.0f);  // Blue
  } else if (lanelet_type == "bicycle_lane") {
    return makeColor(0.0f, 0.8f, 0.0f);  // Green
  }
  return makeColor(0.7f, 0.7f, 0.7f);  // Gray (default)
}

visualization_msgs::msg::Marker createLineStripMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;

  marker.color = color;
  marker.points = points;

  return marker;
}

visualization_msgs::msg::Marker createSphereMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double radius)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = radius * 2.0;
  marker.scale.y = radius * 2.0;
  marker.scale.z = radius * 2.0;

  marker.color = color;

  return marker;
}

visualization_msgs::msg::Marker createDeleteAllMarker(const std::string & ns, const std::string & frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

std_msgs::msg::ColorRGBA getColorForBoundary(uint8_t boundary_type, uint8_t boundary_color)
{
  // Yellow for yellow markings (typically center dividers)
  if (boundary_color == 1) {  // COLOR_YELLOW
    return makeColor(1.0f, 0.9f, 0.0f);  // Yellow
  }
  // White for regular markings, slightly dimmer for dashed
  if (boundary_type == 0) {  // BOUNDARY_DASHED
    return makeColor(0.9f, 0.9f, 0.9f);  // Light gray/white
  }
  return makeColor(1.0f, 1.0f, 1.0f);  // White
}

visualization_msgs::msg::Marker createDashedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width,
  double dash_length,
  double gap_length)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // Pairs of points
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color = color;

  if (points.size() < 2) {
    return marker;
  }

  // Walk along the polyline and emit dash segments
  double accumulated = 0.0;
  bool in_dash = true;
  geometry_msgs::msg::Point dash_start = points[0];

  for (size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].x - points[i - 1].x;
    double dy = points[i].y - points[i - 1].y;
    double dz = points[i].z - points[i - 1].z;
    double segment_len = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (segment_len < 1e-6) {
      continue;
    }

    double pos = 0.0;
    while (pos < segment_len) {
      double target = in_dash ? dash_length : gap_length;
      double remaining_in_mode = target - accumulated;
      double remaining_in_segment = segment_len - pos;

      if (remaining_in_segment >= remaining_in_mode) {
        // Mode ends within this segment
        double t = (pos + remaining_in_mode) / segment_len;
        geometry_msgs::msg::Point pt;
        pt.x = points[i - 1].x + t * dx;
        pt.y = points[i - 1].y + t * dy;
        pt.z = points[i - 1].z + t * dz;

        if (in_dash) {
          marker.points.push_back(dash_start);
          marker.points.push_back(pt);
        } else {
          dash_start = pt;
        }

        in_dash = !in_dash;
        accumulated = 0.0;
        pos += remaining_in_mode;
      } else {
        accumulated += remaining_in_segment;
        pos = segment_len;
      }
    }
  }

  // Close final dash if still in dash mode
  if (in_dash && !points.empty()) {
    marker.points.push_back(dash_start);
    marker.points.push_back(points.back());
  }

  return marker;
}

visualization_msgs::msg::Marker createDottedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double dot_size,
  double dot_spacing)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = dot_size;
  marker.scale.y = dot_size;
  marker.scale.z = dot_size;
  marker.color = color;

  if (points.size() < 2) {
    return marker;
  }

  // Add first point
  marker.points.push_back(points[0]);

  // Walk along the polyline and place dots at regular intervals
  double accumulated = 0.0;

  for (size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].x - points[i - 1].x;
    double dy = points[i].y - points[i - 1].y;
    double dz = points[i].z - points[i - 1].z;
    double segment_len = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (segment_len < 1e-6) {
      continue;
    }

    double pos = 0.0;
    while (pos < segment_len) {
      double remaining_to_next_dot = dot_spacing - accumulated;
      double remaining_in_segment = segment_len - pos;

      if (remaining_in_segment >= remaining_to_next_dot) {
        // Place a dot within this segment
        double t = (pos + remaining_to_next_dot) / segment_len;
        geometry_msgs::msg::Point pt;
        pt.x = points[i - 1].x + t * dx;
        pt.y = points[i - 1].y + t * dy;
        pt.z = points[i - 1].z + t * dz;
        marker.points.push_back(pt);

        accumulated = 0.0;
        pos += remaining_to_next_dot;
      } else {
        accumulated += remaining_in_segment;
        pos = segment_len;
      }
    }
  }

  return marker;
}

visualization_msgs::msg::Marker createArrowMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const std_msgs::msg::ColorRGBA & color,
  double shaft_diameter,
  double head_diameter,
  double head_length)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.points.push_back(start);
  marker.points.push_back(end);

  marker.scale.x = shaft_diameter;
  marker.scale.y = head_diameter;
  marker.scale.z = head_length;

  marker.color = color;

  return marker;
}

visualization_msgs::msg::Marker createTextMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & color,
  double text_height)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = position;
  marker.pose.position.z += 1.0;  // Lift text above ground
  marker.pose.orientation.w = 1.0;

  marker.scale.z = text_height;
  marker.color = color;
  marker.text = text;

  return marker;
}

visualization_msgs::msg::Marker createTriangleMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double size)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = color;

  // Create an upward-pointing triangle (yield sign shape)
  double half = size / 2.0;
  double height = size * 0.866;  // sqrt(3)/2

  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = position.x;
  p1.y = position.y + height * 2.0 / 3.0;
  p1.z = position.z + 0.5;

  p2.x = position.x - half;
  p2.y = position.y - height / 3.0;
  p2.z = position.z + 0.5;

  p3.x = position.x + half;
  p3.y = position.y - height / 3.0;
  p3.z = position.z + 0.5;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);

  return marker;
}

visualization_msgs::msg::Marker createCubeMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & size,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = pose;
  marker.scale.x = size.x;
  marker.scale.y = size.y;
  marker.scale.z = size.z;
  marker.color = color;

  return marker;
}

}  // namespace lanelet_markers
