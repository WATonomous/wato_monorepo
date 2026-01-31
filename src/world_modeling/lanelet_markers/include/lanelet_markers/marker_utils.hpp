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

#ifndef lanelet_markers__MARKER_UTILS_HPP_
#define lanelet_markers__MARKER_UTILS_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace lanelet_markers
{

/// Creates a ColorRGBA message from RGBA values (0.0-1.0 range).
std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);

/// Returns a color based on lanelet type (e.g., "road", "crosswalk").
std_msgs::msg::ColorRGBA getColorForLaneletType(const std::string & lanelet_type);

/// Creates a LINE_STRIP marker from a sequence of points.
visualization_msgs::msg::Marker createLineStripMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width);

/// Creates a SPHERE marker at a given position.
visualization_msgs::msg::Marker createSphereMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double radius);

/// Creates a DELETEALL marker to clear all markers in a namespace.
visualization_msgs::msg::Marker createDeleteAllMarker(const std::string & ns, const std::string & frame_id = "map");

/// Returns a color based on boundary type and color from lanelet message.
std_msgs::msg::ColorRGBA getColorForBoundary(uint8_t boundary_type, uint8_t boundary_color);

/// Creates a dashed line marker using LINE_LIST with alternating segments.
visualization_msgs::msg::Marker createDashedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width,
  double dash_length = 1.0,
  double gap_length = 1.0);

/// Creates a dotted line marker using SPHERE_LIST with dots at regular intervals.
visualization_msgs::msg::Marker createDottedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double dot_size,
  double dot_spacing = 1.0);

/// Creates an ARROW marker pointing from start to end position.
visualization_msgs::msg::Marker createArrowMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const std_msgs::msg::ColorRGBA & color,
  double shaft_diameter = 0.15,
  double head_diameter = 0.3,
  double head_length = 0.4);

/// Creates a TEXT_VIEW_FACING marker at a given position.
visualization_msgs::msg::Marker createTextMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & color,
  double text_height);

/// Creates a TRIANGLE_LIST marker (yield sign shape) at a given position.
visualization_msgs::msg::Marker createTriangleMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double size);

/// Creates a CUBE marker with given pose and size (oriented bounding box).
visualization_msgs::msg::Marker createCubeMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & size,
  const std_msgs::msg::ColorRGBA & color);

}  // namespace lanelet_markers

#endif  // lanelet_markers__MARKER_UTILS_HPP_
