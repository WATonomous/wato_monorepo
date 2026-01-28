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

/**
 * @file map_interface.hpp
 * @brief HD MAP QUERY WRAPPER - Provides map information to trajectory
 * generation
 * * WHAT THIS FILE DOES:
 * - Wraps service calls to the lanelet map server
 * - Finds nearest lanelet to object position
 * - Gets lanelet centerlines (path points for trajectory following)
 * - Gets possible future lanelets (for generating multiple hypotheses)
 * - Checks for crosswalks (important for pedestrians and cyclists)
 * - Gets speed limits and regulatory elements
 * * WHO WORKS ON THIS:
 * - Sean Yang
 * * CURRENT STATUS:
 * - First draft complete
 * - Currently does not have tests
 */

#ifndef PREDICTION__MAP_INTERFACE_HPP_
#define PREDICTION__MAP_INTERFACE_HPP_

#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"
#include "rclcpp/rclcpp.hpp"

namespace prediction {

/**
 * @brief Lanelet information structure
 */
struct LaneletInfo {
  int64_t id;
  std::vector<geometry_msgs::msg::Point> centerline;
  double speed_limit;
  std::vector<int64_t> following_lanelets;
  std::vector<int64_t> previous_lanelets;
};

/**
 * @brief Interface to HD map for lanelet queries
 * * Wraps service calls to the lanelet map server for:
 * - Finding nearest lanelets
 * - Querying lanelet information
 * - Computing route connectivity
 * - Getting regulatory elements
 */
class MapInterface {
public:
  /**
   * @brief Construct a new Map Interface
   * @param node ROS node pointer for service clients
   */
  explicit MapInterface(rclcpp::Node *node);

  /**
   * @brief Find nearest lanelet to a point
   * @param point Query point
   * @return Lanelet ID of nearest lanelet, or nullopt if none found
   */
  std::optional<int64_t>
  findNearestLanelet(const geometry_msgs::msg::Point &point);

  /**
   * @brief Get lanelet information by ID
   * @param lanelet_id Lanelet ID
   * @return LaneletInfo structure, or nullopt if not found
   */
  std::optional<LaneletInfo> getLaneletById(int64_t lanelet_id);

  /**
   * @brief Get possible future lanelets from current lanelet
   * @param current_lanelet_id Current lanelet ID
   * @param max_depth Maximum search depth
   * @return Vector of possible future lanelet IDs
   */
  std::vector<int64_t> getPossibleFutureLanelets(int64_t current_lanelet_id,
                                                 int max_depth = 3);

  /**
   * @brief Get speed limit for a lanelet
   * @param lanelet_id Lanelet ID
   * @return Speed limit in m/s, or nullopt if not found
   */
  std::optional<double> getSpeedLimit(int64_t lanelet_id);

  /**
   * @brief Check if a crosswalk is near a point
   * @param point Query point
   * @param radius Search radius
   * @return True if crosswalk nearby
   */
  bool isCrosswalkNearby(const geometry_msgs::msg::Point &point, double radius);

private:
  /**
   * @brief Callback for lane context topic
   * @param msg CurrentLaneContext message
   */
  void laneContextCallback(
      const lanelet_msgs::msg::CurrentLaneContext::SharedPtr msg);

  /**
   * @brief Callback for route ahead topic
   * @param msg RouteAhead message
   */
  void routeAheadCallback(const lanelet_msgs::msg::RouteAhead::SharedPtr msg);

  /**
   * @brief Cache a lanelet message
   * @param lanelet Lanelet message to cache
   */
  void cacheLanelet(const lanelet_msgs::msg::Lanelet &lanelet);

  /**
   * @brief Cache multiple lanelet messages
   * @param lanelets Vector of Lanelet messages to cache
   */
  void cacheLanelets(const std::vector<lanelet_msgs::msg::Lanelet> &lanelets);

  /**
   * @brief Convert lanelet_msgs::Lanelet to LaneletInfo
   * @param lanelet Lanelet message
   * @return LaneletInfo structure
   */
  LaneletInfo laneletMsgToInfo(const lanelet_msgs::msg::Lanelet &lanelet) const;

  /**
   * @brief Compute distance from a point to a lanelet centerline
   * @param point Query point
   * @param lanelet Lanelet to check
   * @return Minimum distance to centerline
   */
  double distanceToLanelet(const geometry_msgs::msg::Point &point,
                           const lanelet_msgs::msg::Lanelet &lanelet) const;

  rclcpp::Node *node_;

  // Subscriptions
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr
      lane_context_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr
      route_ahead_sub_;

  // Cached data
  // Store pair of Lanelet and iterator to LRU list
  struct CachedLanelet {
    lanelet_msgs::msg::Lanelet lanelet;
    std::list<int64_t>::iterator lru_iterator;
  };
  std::unordered_map<int64_t, CachedLanelet> lanelet_cache_;
  std::list<int64_t> lru_list_;
  static constexpr size_t MAX_CACHE_SIZE = 1000;
  mutable std::mutex cache_mutex_;

  // Current lane context
  lanelet_msgs::msg::CurrentLaneContext::SharedPtr current_lane_context_;
  mutable std::mutex context_mutex_;
};

} // namespace prediction

#endif // PREDICTION__MAP_INTERFACE_HPP_
