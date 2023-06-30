#ifndef WORLD_MODELING_LANELET_VISUALIZATION_
#define WORLD_MODELING_LANELET_VISUALIZATION_

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace world_modeling::hd_map{
    visualization_msgs::msg::MarkerArray laneletMapAsMarkerArray(lanelet::LaneletMapPtr map);
    visualization_msgs::msg::MarkerArray laneletAsMarkerArray(lanelet::Lanelet lanelet, int *id);
    visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(lanelet::LineStringLayer& lineStrings);
    visualization_msgs::msg::Marker lineStringAsMarker(lanelet::ConstLineString3d lineString, int *id, float thickness, int type, std_msgs::msg::ColorRGBA color);
}

#endif