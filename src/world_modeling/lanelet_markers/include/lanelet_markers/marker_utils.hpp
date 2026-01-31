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

/**
 * @brief Creates a ColorRGBA message from RGBA float values.
 *
 * @param r Red channel (0.0 - 1.0).
 * @param g Green channel (0.0 - 1.0).
 * @param b Blue channel (0.0 - 1.0).
 * @param a Alpha channel (0.0 - 1.0, default fully opaque).
 * @return Populated ColorRGBA message.
 */
std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);

/**
 * @brief Returns a color based on lanelet type string.
 *
 * Maps types like "road", "crosswalk", "bicycle_lane", etc. to distinct colors
 * for map visualization. Falls back to white for unknown types.
 *
 * @param lanelet_type Lanelet subtype attribute value.
 * @return RGBA color for the given lanelet type.
 */
std_msgs::msg::ColorRGBA getColorForLaneletType(const std::string & lanelet_type);

/**
 * @brief Creates a LINE_STRIP marker from a sequence of points.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param points Ordered points forming the line strip.
 * @param color RGBA color of the line.
 * @param line_width Width of the line strip in meters.
 * @return Configured LINE_STRIP marker.
 */
visualization_msgs::msg::Marker createLineStripMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width);

/**
 * @brief Creates a SPHERE marker at a given position.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param position Center position of the sphere.
 * @param color RGBA color of the sphere.
 * @param radius Sphere radius in meters.
 * @return Configured SPHERE marker.
 */
visualization_msgs::msg::Marker createSphereMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double radius);

/**
 * @brief Creates a DELETEALL marker to clear all markers in a namespace.
 *
 * @param ns Marker namespace to clear.
 * @param frame_id TF frame for the marker header.
 * @return DELETEALL marker that removes all markers in the given namespace.
 */
visualization_msgs::msg::Marker createDeleteAllMarker(const std::string & ns, const std::string & frame_id = "map");

/**
 * @brief Returns a color based on boundary type and color from a lanelet message.
 *
 * Maps boundary type constants (BOUNDARY_SOLID, BOUNDARY_DASHED, etc.) and
 * color constants (COLOR_WHITE, COLOR_YELLOW) to appropriate RGBA values.
 *
 * @param boundary_type Boundary type constant from lanelet_msgs::msg::Lanelet.
 * @param boundary_color Boundary color constant from lanelet_msgs::msg::Lanelet.
 * @return RGBA color for the given boundary.
 */
std_msgs::msg::ColorRGBA getColorForBoundary(uint8_t boundary_type, uint8_t boundary_color);

/**
 * @brief Creates a dashed line marker using LINE_LIST with alternating visible segments.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param points Ordered points defining the line path.
 * @param color RGBA color of the dashes.
 * @param line_width Width of each dash in meters.
 * @param dash_length Length of each visible dash segment in meters.
 * @param gap_length Length of each gap between dashes in meters.
 * @return Configured LINE_LIST marker with dashed pattern.
 */
visualization_msgs::msg::Marker createDashedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double line_width,
  double dash_length = 1.0,
  double gap_length = 1.0);

/**
 * @brief Creates a dotted line marker using SPHERE_LIST with dots at regular intervals.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param points Ordered points defining the line path to sample dots along.
 * @param color RGBA color of the dots.
 * @param dot_size Diameter of each dot in meters.
 * @param dot_spacing Distance between dot centers in meters.
 * @return Configured SPHERE_LIST marker with dotted pattern.
 */
visualization_msgs::msg::Marker createDottedLineMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double dot_size,
  double dot_spacing = 1.0);

/**
 * @brief Creates an ARROW marker pointing from a start to an end position.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param start Arrow tail position.
 * @param end Arrow head position.
 * @param color RGBA color of the arrow.
 * @param shaft_diameter Diameter of the arrow shaft in meters.
 * @param head_diameter Diameter of the arrow head in meters.
 * @param head_length Length of the arrow head in meters.
 * @return Configured ARROW marker.
 */
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

/**
 * @brief Creates a TEXT_VIEW_FACING marker at a given position.
 *
 * The text always faces the camera regardless of viewpoint.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param position Position to place the text anchor.
 * @param text String content to display.
 * @param color RGBA color of the text.
 * @param text_height Height of the text in meters (controls font size in RViz).
 * @return Configured TEXT_VIEW_FACING marker.
 */
visualization_msgs::msg::Marker createTextMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & color,
  double text_height);

/**
 * @brief Creates a TRIANGLE_LIST marker shaped as a yield sign at a given position.
 *
 * Renders an inverted triangle (yield sign) using three vertices.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param position Center position of the yield sign.
 * @param color RGBA color of the triangle.
 * @param size Scale of the triangle in meters.
 * @return Configured TRIANGLE_LIST marker.
 */
visualization_msgs::msg::Marker createTriangleMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double size);

/**
 * @brief Creates a CUBE marker with a given pose and size for oriented bounding boxes.
 *
 * @param ns Marker namespace for grouping and deletion.
 * @param id Unique marker ID within the namespace.
 * @param frame_id TF frame the marker is expressed in.
 * @param pose Position and orientation of the cube center.
 * @param size Dimensions of the cube (x, y, z) in meters.
 * @param color RGBA color of the cube.
 * @return Configured CUBE marker.
 */
visualization_msgs::msg::Marker createCubeMarker(
  const std::string & ns,
  int32_t id,
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & size,
  const std_msgs::msg::ColorRGBA & color);

}  // namespace lanelet_markers

#endif  // lanelet_markers__MARKER_UTILS_HPP_
