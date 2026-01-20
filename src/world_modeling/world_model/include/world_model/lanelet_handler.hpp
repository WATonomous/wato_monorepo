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

#ifndef WORLD_MODEL__LANELET_HANDLER_HPP_
#define WORLD_MODEL__LANELET_HANDLER_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/corridor.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/map_visualization.hpp"
#include "lanelet_msgs/srv/get_corridor.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"
#include "lanelet_msgs/srv/get_route.hpp"
#include "rclcpp/rclcpp.hpp"

namespace world_model
{

class LaneletHandler
{
public:
  LaneletHandler();
  ~LaneletHandler() = default;

  /**
   * Load an OSM lanelet map with UTM projection.
   *
   * @param osm_path Path to the .osm file
   * @param lat_origin Latitude origin for UTM projection
   * @param lon_origin Longitude origin for UTM projection
   * @return true if map loaded successfully
   */
  bool loadMap(const std::string & osm_path, double lat_origin, double lon_origin);

  /**
   * Check if the map is loaded.
   */
  bool isMapLoaded() const;

  // Service implementations
  lanelet_msgs::srv::GetRoute::Response getRoute(int64_t from_id, int64_t to_id);
  lanelet_msgs::srv::GetCorridor::Response getCorridor(
    int64_t from_id, int64_t to_id, double max_length_m,
    double sample_spacing_m, int32_t num_lanes_each_side);
  lanelet_msgs::srv::GetLaneletsByRegElem::Response getLaneletsByRegElem(int64_t reg_elem_id);

  // Query methods
  std::optional<lanelet::ConstLanelet> findNearestLanelet(const geometry_msgs::msg::Point & point);
  std::optional<lanelet::ConstLanelet> getLaneletById(int64_t id);
  std::vector<lanelet::ConstLanelet> getLaneletsInRadius(
    const geometry_msgs::msg::Point & center, double radius);

  // Conversion methods
  lanelet_msgs::msg::Lanelet toLaneletMsg(const lanelet::ConstLanelet & ll);
  lanelet_msgs::msg::Corridor toCorridorMsg(
    const std::vector<lanelet::ConstLanelet> & route_lanelets,
    int32_t num_lanes_each_side, double sample_spacing_m, double max_length_m);

  // Routing graph access
  lanelet::routing::RoutingGraphConstPtr getRoutingGraph() const;
  lanelet::traffic_rules::TrafficRulesPtr getTrafficRules() const;

private:
  lanelet::LaneletMapPtr map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  std::unique_ptr<lanelet::projection::UtmProjector> projector_;

  // Helper methods
  void populateLaneletSemantics(
    lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll);
  void populateLaneletConnectivity(
    lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll);
  void populateLaneletRegulatoryElements(
    lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll);

  std::vector<geometry_msgs::msg::Point> sampleCenterline(
    const lanelet::ConstLanelet & ll, double spacing_m);
  uint8_t getTransitionType(
    const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to);
};

}  // namespace world_model

#endif  // WORLD_MODEL__LANELET_HANDLER_HPP_
