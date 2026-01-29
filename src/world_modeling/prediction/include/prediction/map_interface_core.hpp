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
 * @file map_interface_core.hpp
 * @brief Core logic for HD MAP queries - separated from ROS2 dependencies
 *
 * This file contains the pure business logic for map interface operations:
 * - LRU caching of lanelets
 * - Distance calculations
 * - Finding nearest lanelets
 * - BFS for future lanelet exploration
 *
 * The separation allows for unit testing without ROS2 infrastructure.
 */

#ifndef PREDICTION__MAP_INTERFACE_CORE_HPP_
#define PREDICTION__MAP_INTERFACE_CORE_HPP_

#include <cmath>
#include <limits>
#include <list>
#include <mutex>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace prediction
{

/**
 * @brief Lanelet information structure
 */
struct LaneletInfo
{
  int64_t id;
  std::vector<geometry_msgs::msg::Point> centerline;
  double speed_limit;
  std::vector<int64_t> following_lanelets;
  std::vector<int64_t> previous_lanelets;
};

/**
 * @brief Core logic for HD map interface operations
 *
 * This class contains the pure business logic without ROS2 dependencies,
 * enabling unit testing without spinning up ROS2 nodes.
 */
class MapInterfaceCore
{
public:
  static constexpr size_t DEFAULT_MAX_CACHE_SIZE = 1000;

  explicit MapInterfaceCore(size_t max_cache_size = DEFAULT_MAX_CACHE_SIZE)
  : max_cache_size_(max_cache_size)
  {}

  /**
   * @brief Cache a lanelet message
   * @param lanelet Lanelet message to cache
   */
  void cacheLanelet(const lanelet_msgs::msg::Lanelet & lanelet);

  /**
   * @brief Cache multiple lanelet messages
   * @param lanelets Vector of Lanelet messages to cache
   */
  void cacheLanelets(const std::vector<lanelet_msgs::msg::Lanelet> & lanelets);

  /**
   * @brief Find nearest lanelet to a point
   * @param point Query point
   * @return Lanelet ID of nearest lanelet, or nullopt if cache is empty
   */
  std::optional<int64_t> findNearestLanelet(const geometry_msgs::msg::Point & point);

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
  std::vector<int64_t> getPossibleFutureLanelets(int64_t current_lanelet_id, int max_depth = 3);

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
  bool isCrosswalkNearby(const geometry_msgs::msg::Point & point, double radius);

  /**
   * @brief Check if cache is empty
   * @return True if no lanelets are cached
   */
  bool isCacheEmpty() const;

  /**
   * @brief Get the number of cached lanelets
   * @return Number of lanelets in cache
   */
  size_t getCacheSize() const;

  /**
   * @brief Clear all cached lanelets
   */
  void clearCache();

  /**
   * @brief Convert lanelet_msgs::Lanelet to LaneletInfo
   * @param lanelet Lanelet message
   * @return LaneletInfo structure
   */
  static LaneletInfo laneletMsgToInfo(const lanelet_msgs::msg::Lanelet & lanelet);

  /**
   * @brief Compute distance from a point to a lanelet centerline
   * @param point Query point
   * @param lanelet Lanelet to check
   * @return Minimum distance to centerline
   */
  static double distanceToLanelet(const geometry_msgs::msg::Point & point, const lanelet_msgs::msg::Lanelet & lanelet);

  /**
   * @brief Compute distance from a point to a line segment
   * @param p Query point
   * @param a Segment start point
   * @param b Segment end point
   * @return Distance from point to segment
   */
  static double distancePointToSegment(
    const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

  /**
   * @brief Get the maximum cache size
   * @return Maximum number of lanelets that can be cached
   */
  size_t getMaxCacheSize() const
  {
    return max_cache_size_;
  }

private:
  struct CachedLanelet
  {
    lanelet_msgs::msg::Lanelet lanelet;
    std::list<int64_t>::iterator lru_iterator;
  };

  size_t max_cache_size_;
  std::unordered_map<int64_t, CachedLanelet> lanelet_cache_;
  std::list<int64_t> lru_list_;
  mutable std::mutex cache_mutex_;
};

}  // namespace prediction

#endif  // PREDICTION__MAP_INTERFACE_CORE_HPP_
