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
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
#include "lanelet_msgs/srv/get_lanelets_by_reg_elem.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

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

  /**
   * @brief Find the nearest lanelet to a point by 2D distance.
   *
   * Iterates all lanelets in the map and returns the one with the smallest
   * 2D distance to the query point.
   *
   * @param point Query position in the map frame.
   * @return Nearest lanelet, or nullopt if no map is loaded.
   */
  std::optional<lanelet::ConstLanelet> findNearestLanelet(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Find the ID of the nearest lanelet to a point.
   *
   * @param point Query position in the map frame.
   * @return Nearest lanelet ID, or nullopt if no map is loaded.
   */
  std::optional<int64_t> findNearestLaneletId(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Find current lanelet using smart selection.
   *
   * Priority:
   * 1. If active route exists, return route lanelet that ego is on
   * 2. If previous_lanelet_id is provided and a routing graph exists,
   *    BFS 1-level from previous lanelet (following, left, right + self),
   *    score by heading alignment, return if a good candidate is found
   * 3. Fallback: brute-force radius search with heading alignment
   *
   * @param point Ego position
   * @param heading_rad Ego heading (yaw) in radians
   * @param route_priority_threshold_m Max distance to consider ego "on" a route lanelet
   * @param heading_search_radius_m Radius to search for heading-aligned lanelets
   * @param previous_lanelet_id Optional hint: last known lanelet for BFS neighbor search
   * @return Lanelet ID if found
   */
  std::optional<int64_t> findCurrentLaneletId(
    const geometry_msgs::msg::Point & point,
    double heading_rad,
    double route_priority_threshold_m = 10.0,
    double heading_search_radius_m = 15.0,
    std::optional<int64_t> previous_lanelet_id = std::nullopt) const;

  /**
   * @brief Retrieve a lanelet by its unique ID.
   *
   * @param id Lanelet ID to look up.
   * @return The lanelet if found, or nullopt if the ID doesn't exist or no map is loaded.
   */
  std::optional<lanelet::ConstLanelet> getLaneletById(int64_t id) const;

  /**
   * @brief Find all lanelets within a given radius of a point.
   *
   * @param center Center position in the map frame.
   * @param radius Search radius in meters.
   * @return Vector of lanelets whose 2D distance to center is within the radius.
   */
  std::vector<lanelet::ConstLanelet> getLaneletsInRadius(const geometry_msgs::msg::Point & center, double radius) const;

  // Route Caching (for SetRoute/GetShortestRoute workflow)

  /**
   * @brief Set and cache an active route from current to goal lanelet.
   *
   * Computes the shortest path and caches it for subsequent getShortestRoute() calls.
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
   * @brief Get the entire shortest route from current position to goal.
   *
   * Finds ego's current position on the cached route and returns ALL lanelets
   * from that position to the goal.
   *
   * @param current_pos Current ego position
   * @return GetShortestRoute response with all lanelets to goal
   */
  lanelet_msgs::srv::GetShortestRoute::Response getShortestRoute(const geometry_msgs::msg::Point & current_pos) const;

  /**
   * @brief Get route lanelets ahead of ego within a lookahead distance.
   *
   * Finds ego's current position on the cached route and returns lanelets
   * within the specified lookahead distance. Used by RouteAhead publisher.
   *
   * @param current_pos Current ego position
   * @param lookahead_distance_m Maximum distance along route to include
   * @return RouteAhead message with lanelets within lookahead distance
   */
  lanelet_msgs::msg::RouteAhead getRouteAhead(
    const geometry_msgs::msg::Point & current_pos, double lookahead_distance_m) const;

  /**
   * @brief Get legally reachable lanelets ahead of ego within a radius.
   *
   * Uses BFS through the routing graph starting from the current lanelet,
   * expanding via following (successors) and left/right (legal lane changes).
   * Only returns forward-reachable, same-direction, legally-connected lanelets
   * within the specified radius. Falls back to just the current lanelet if
   * no routing graph is available.
   *
   * @param current_pos Current ego position
   * @param heading_rad Current ego heading (yaw) in radians for current lanelet detection
   * @param radius_m Radius bound for BFS expansion
   * @param previous_lanelet_id Optional hint: last known lanelet for BFS neighbor search
   * @param route_priority_threshold_m Max distance to prefer a route lanelet
   * @param heading_search_radius_m Radius to search for heading-aligned lanelets
   * @return LaneletAhead message with reachable lanelets within radius
   */
  lanelet_msgs::msg::LaneletAhead getLaneletAhead(
    const geometry_msgs::msg::Point & current_pos,
    double heading_rad,
    double radius_m,
    std::optional<int64_t> previous_lanelet_id = std::nullopt,
    double route_priority_threshold_m = 10.0,
    double heading_search_radius_m = 15.0) const;

  /**
   * @brief Find the nearest traffic light regulatory element to a point.
   *
   * Iterates all regulatory elements with subtype "traffic_light",
   * computes distance from their "refers" positions to the query point,
   * and returns the ID of the closest one.
   */
  std::optional<int64_t> findNearestTrafficLightRegElemId(const geometry_msgs::msg::Point & point) const;

  // Service implementations

  /**
   * @brief Find all lanelets that reference a given regulatory element.
   *
   * @param reg_elem_id Regulatory element ID to search for.
   * @return Response with matching lanelet messages and success flag.
   */
  lanelet_msgs::srv::GetLaneletsByRegElem::Response getLaneletsByRegElem(int64_t reg_elem_id) const;

  // Conversions

  /**
   * @brief Convert a Lanelet2 ConstLanelet to a ROS lanelet message.
   *
   * Populates boundaries, centerline, curvature, boundary attributes,
   * semantics, connectivity, and regulatory elements.
   *
   * @param ll Source lanelet from the Lanelet2 map.
   * @return Fully populated lanelet message.
   */
  lanelet_msgs::msg::Lanelet toLaneletMsg(const lanelet::ConstLanelet & ll) const;

  // Accessors

  /// @brief Returns the routing graph (may be null if map not loaded).
  lanelet::routing::RoutingGraphConstPtr getRoutingGraph() const
  {
    return routing_graph_;
  }

  /// @brief Returns the traffic rules used for speed limits and lane changes.
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

  /**
   * @brief Populate lanelet type, intersection flag, and speed limit fields.
   *
   * @param msg Output lanelet message to populate.
   * @param ll Source lanelet to extract attributes from.
   */
  void populateLaneletSemantics(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  /**
   * @brief Populate successor, left, and right lane connectivity fields.
   *
   * Uses the routing graph to determine legal lane connections.
   *
   * @param msg Output lanelet message to populate.
   * @param ll Source lanelet to query connectivity for.
   */
  void populateLaneletConnectivity(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  /**
   * @brief Populate regulatory element fields (traffic lights, stop lines, yield).
   *
   * Extracts referred geometry positions, reference lines with arc-length distances,
   * yield/right-of-way relationships, and generic attributes from each regulatory element.
   *
   * @param msg Output lanelet message to populate.
   * @param ll Source lanelet to extract regulatory elements from.
   */
  void populateLaneletRegulatoryElements(lanelet_msgs::msg::Lanelet & msg, const lanelet::ConstLanelet & ll) const;

  /**
   * @brief Map a boundary linestring's subtype to a visualization type constant.
   *
   * @param boundary Lanelet boundary linestring with "subtype" or "type" attribute.
   * @return Boundary type constant (BOUNDARY_SOLID, BOUNDARY_DASHED, etc.).
   */
  uint8_t getBoundaryTypeForVisualization(const lanelet::ConstLineString3d & boundary) const;

  /**
   * @brief Map a boundary linestring's color attribute to a color constant.
   *
   * @param boundary Lanelet boundary linestring with optional "color" attribute.
   * @return COLOR_YELLOW if color is "yellow", COLOR_WHITE otherwise.
   */
  uint8_t getBoundaryColor(const lanelet::ConstLineString3d & boundary) const;

  /**
   * @brief Get the speed limit for a lanelet using traffic rules.
   *
   * @param ll Lanelet to query speed limit for.
   * @return Speed limit in m/s, or 0.0 if no traffic rules are available.
   */
  double getSpeedLimit(const lanelet::ConstLanelet & ll) const;

  /**
   * @brief BFS through routing graph to find reachable lanelets within radius.
   *
   * Expands via following() (successors), left(), and right() (legal lane changes).
   * Visited set prevents infinite loops on circular roads.
   *
   * @param start Starting lanelet for BFS
   * @param center Center point for radius check
   * @param radius Maximum distance from center for a lanelet to be included
   * @return Vector of reachable lanelets within radius
   */
  std::vector<lanelet::ConstLanelet> getReachableLaneletsInRadius(
    const lanelet::ConstLanelet & start, const lanelet::BasicPoint2d & center, double radius) const;
};

}  // namespace world_model

#endif  // WORLD_MODEL__LANELET_HANDLER_HPP_
