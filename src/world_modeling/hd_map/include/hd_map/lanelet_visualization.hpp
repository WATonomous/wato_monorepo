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

#ifndef WORLD_MODELING_LANELET_VISUALIZATION_
#define WORLD_MODELING_LANELET_VISUALIZATION_

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>  // For std::max
#include <limits>  // For std::numeric_limits
#include <vector>  // For std::vector

#include "geometry_msgs/msg/point.hpp"
#include "hd_map/pedestrian_reg_elem.hpp"
#include "hd_map/traffic_light_reg_elem.hpp"
#include "hd_map/traffic_sign_reg_elem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace world_modeling::hd_map
{

/**
 * Convert a lanelet map to visualization markers.
 *
 * @param map the lanelet map to convert
 * @return MarkerArray for RViz visualization
 */
visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map);

/**
 * Convert a lanelet path to visualization markers.
 *
 * @param laneletPath the lanelet path to convert
 * @return MarkerArray for RViz visualization
 */
visualization_msgs::msg::MarkerArray laneletPathAsMarkerArray(lanelet::routing::LaneletPath laneletPath);

/**
 * Convert a lanelet to visualization markers.
 *
 * @param lanelet the lanelet to visualize
 * @param id pointer to marker ID
 * @param center whether to show centerline
 * @param lanes whether to show lane boundaries
 * @param centerColor colour for centerline
 * @param laneColor colour for lane boundaries
 * @param centerThickness thickness of centerline
 * @param laneThickness thickness of lane boundaries
 * @return MarkerArray for RViz visualization
 */
visualization_msgs::msg::MarkerArray laneletAsMarkerArray(
  lanelet::ConstLanelet lanelet,
  int * id,
  bool center = false,
  bool lanes = true,
  std_msgs::msg::ColorRGBA centerColor = std_msgs::msg::ColorRGBA(),
  std_msgs::msg::ColorRGBA laneColor = std_msgs::msg::ColorRGBA(),
  float centerThickness = .2,
  float laneThickness = .2);
visualization_msgs::msg::MarkerArray laneletAsMarkerArray(
  lanelet::Lanelet lanelet,
  int * id,
  bool center = false,
  bool lanes = true,
  std_msgs::msg::ColorRGBA centerColor = std_msgs::msg::ColorRGBA(),
  std_msgs::msg::ColorRGBA laneColor = std_msgs::msg::ColorRGBA(),
  float centerThickness = .2,
  float laneThickness = .2);

/**
 * Convert line strings to visualization markers.
 *
 * @param lineStrings the line string layer to visualize
 * @return MarkerArray for RViz visualization
 */
visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(const lanelet::LineStringLayer & lineStrings);

/**
 * Convert a line string to visualization marker.
 *
 * @param lineString the line string layer to visualize
 * @param id pointer to marker ID
 * @param thickness line thickness
 * @param type marker type
 * @param color marker color
 * @return Marker for RViz visualization
 */
visualization_msgs::msg::Marker lineStringAsMarker(
  const lanelet::ConstLineString3d lineString, int * id, float thickness, int type, std_msgs::msg::ColorRGBA color);

/**
 * Convert a polygon to visualization markers.
 *
 * @param polygon the polygon to visualize
 * @param id pointer to marker ID
 * @param thickness line thickness
 * @param type marker type
 * @param color marker color
 * @return Marker for RViz visualization
 */
visualization_msgs::msg::Marker polygonToMarker(
  lanelet::ConstPolygon3d polygon, uint64_t * id, float thickness, int type, std_msgs::msg::ColorRGBA color);

/**
 * Convert Traffic lights to visualization markers.
 *
 * @param trafficLightRegElems the traffic light to visualize
 * @return Marker for RViz visualization
 */
visualization_msgs::msg::MarkerArray trafficLightsAsMakerArray(
  std::vector<std::shared_ptr<TrafficLightRegElem>> trafficLightRegElems);

/**
 * Convert Traffic sign to visualization markers.
 *
 * @param trafficSignRegElems the traffic sign to visualize
 * @return Marker for RViz visualization
 */
visualization_msgs::msg::MarkerArray trafficSignsAsMakerArray(
  std::vector<std::shared_ptr<TrafficSignRegElem>> trafficSignRegElems);

/**
 * Convert pedestrians to visualization markers.
 *
 * @param pedestrianRegElems the pedestrian to visualize
 * @return Marker for RViz visualization
 */
visualization_msgs::msg::MarkerArray pedestrianAsMarkerArray(
  std::vector<std::shared_ptr<PedestrianRegElem>> pedestrianRegElems);

// TODO(wato): Resolve overlapping marker IDs for traffic light markers.
// TODO(wato): Consolidate duplicated logic across lanelet rendering functions.
// TODO(wato): Optimize marker array publishing rate to reduce bandwidth.

}  // namespace world_modeling::hd_map

#endif
