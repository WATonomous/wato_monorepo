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
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/corridor.hpp"
#include "lanelet_msgs/msg/corridor_lane.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/srv/get_corridor.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"
#include "lanelet_msgs/srv/get_route.hpp"

namespace world_model
{

/**
 * @brief Wrapper around Lanelet2 map for queries and service implementations.
 *
 * Thread-safe for concurrent reads after loadMap() completes.
 * Lanelet2 routing graph is immutable after construction.
 */
class LaneletHandler
{
public:
  LaneletHandler() = default;
  ~LaneletHandler() = default;

  // Non-copyable, movable
  LaneletHandler(const LaneletHandler &) = delete;
  LaneletHandler & operator=(const LaneletHandler &) = delete;
  LaneletHandler(LaneletHandler &&) = default;
  LaneletHandler & operator=(LaneletHandler &&) = default;

  /**
   * @brief Load an OSM lanelet map with configurable projection.
   *
   * @param osm_path Path to the .osm file
   * @param utm_origin_x X offset from UTM origin to map frame origin (meters)
   * @param utm_origin_y Y offset from UTM origin to map frame origin (meters)
   * @param projector_type Projector type: "utm" or "local_cartesian"
   * @return true if map loaded successfully
   */
  bool loadMap(
    const std::string & osm_path, double utm_origin_x, double utm_origin_y, const std::string & projector_type = "utm");

  /**
   * @brief Check if the map is loaded.
   */
  bool isMapLoaded() const
  {
    return map_ != nullptr;
  }

  // Queries (all const, thread-safe for concurrent reads)

  std::optional<lanelet::ConstLanelet> findNearestLanelet(const geometry_msgs::msg::Point & point) const;

  std::optional<int64_t> findNearestLaneletId(const geometry_msgs::msg::Point & point) const;

  std::optional<lanelet::ConstLanelet> getLaneletById(int64_t id) const;

  std::vector<lanelet::ConstLanelet> getLaneletsInRadius(const geometry_msgs::msg::Point & center, double radius) const;

  // Route Caching (for SetRoute/GetRoute workflow)

  /**
   * @brief Set and cache an active route from current to goal lanelet.
   *
   * Computes the shortest path and caches it for subsequent getRouteFromPosition() calls.
   *
   * @param from_id Starting lanelet ID (ego's current lanelet)
   * @param to_id Destination lanelet ID (goal lanelet)
   * @return true if route was computed and cached successfully
   */
  bool setActiveRoute(int64_t from_id, int64_t to_id);

  /**
   * @brief Check if an active route is cached.
   */
  bool hasActiveRoute() const;

  /**
   * @brief Clear the cached active route.
   */
  void clearActiveRoute();

  /**
   * @brief Get the goal lanelet ID of the active route.
   * @return Goal lanelet ID, or -1 if no active route
   */
  int64_t getGoalLaneletId() const;

  /**
   * @brief Get route lanelets from current position within a distance.
   *
   * Finds ego's current position on the cached route and returns lanelets
   * within the specified distance along the route.
   *
   * @param current_pos Current ego position
   * @param distance_m Maximum distance along route to include
   * @return GetRoute response with lanelets and remaining distance
   */
  lanelet_msgs::srv::GetRoute::Response getRouteFromPosition(
    const geometry_msgs::msg::Point & current_pos, double distance_m) const;

  // Service implementations

  lanelet_msgs::srv::GetRoute::Response getRoute(int64_t from_id, int64_t to_id) const;

  lanelet_msgs::srv::GetCorridor::Response getCorridor(
    int64_t from_id, int64_t to_id, double max_length_m, double sample_spacing_m) const;

  lanelet_msgs::srv::GetLaneletsByRegElem::Response getLaneletsByRegElem(int64_t reg_elem_id) const;

  // Conversions

  lanelet_msgs::msg::Lanelet toLaneletMsg(const lanelet::ConstLanelet & ll) const;

  lanelet_msgs::msg::Corridor toCorridorMsg(
    const std::vector<lanelet::ConstLanelet> & route_lanelets, double sample_spacing_m, double max_length_m) const;

  // Accessors

  lanelet::routing::RoutingGraphConstPtr getRoutingGraph() const
  {
    return routing_graph_;
  }

  lanelet::traffic_rules::TrafficRulesPtr getTrafficRules() const
  {
    return traffic_rules_;
  }

private:
  lanelet::LaneletMapPtr map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  std::unique_ptr<lanelet::Projector> projector_;

  // Route caching state (protected by mutex for thread safety)
  mutable std::mutex route_mutex_;
  std::vector<lanelet::ConstLanelet> active_route_;
  int64_t goal_lanelet_id_ = -1;

  // Helper methods for Lanelet message
  void populateLaneletSemantics(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  void populateLaneletConnectivity(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  void populateLaneletRegulatoryElements(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  // Helper methods for Corridor message
  lanelet_msgs::msg::CorridorLane buildCorridorLane(
    const std::vector<lanelet::ConstLanelet> & lanelets, double sample_spacing_m, double max_length_m) const;

  uint8_t getBoundaryType(const lanelet::ConstLineString3d & boundary) const;

  uint8_t getBoundaryTypeForVisualization(const lanelet::ConstLineString3d & boundary) const;

  uint8_t getBoundaryColor(const lanelet::ConstLineString3d & boundary) const;

  double getSpeedLimit(const lanelet::ConstLanelet & ll) const;
};

}  // namespace world_model

#endif  // WORLD_MODEL__LANELET_HANDLER_HPP_
