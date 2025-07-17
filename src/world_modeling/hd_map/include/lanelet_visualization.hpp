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

#include "geometry_msgs/msg/point.hpp"
#include "pedestrian_reg_elem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "traffic_light_reg_elem.hpp"
#include "traffic_sign_reg_elem.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace world_modeling::hd_map
{
visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map);
visualization_msgs::msg::MarkerArray laneletPathAsMarkerArray(lanelet::routing::LaneletPath laneletPath);
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
visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(lanelet::LineStringLayer & lineStrings);
visualization_msgs::msg::Marker lineStringAsMarker(
  lanelet::ConstLineString3d lineString, int * id, float thickness, int type, std_msgs::msg::ColorRGBA color);
visualization_msgs::msg::Marker polygonToMarker(
  lanelet::ConstPolygon3d polygon, uint64_t * id, float thickness, int type, std_msgs::msg::ColorRGBA color);
visualization_msgs::msg::MarkerArray trafficLightsAsMakerArray(
  std::vector<std::shared_ptr<TrafficLightRegElem>> trafficLightRegElems);
visualization_msgs::msg::MarkerArray trafficSignsAsMakerArray(
  std::vector<std::shared_ptr<TrafficSignRegElem>> trafficSignRegElems);
visualization_msgs::msg::MarkerArray pedestrianAsMarkerArray(
  std::vector<std::shared_ptr<PedestrianRegElem>> pedestrianRegElems);
}  // namespace world_modeling::hd_map

#endif
