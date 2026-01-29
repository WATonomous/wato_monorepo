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
 * @file map_interface_node.hpp
 * @brief HD MAP QUERY WRAPPER - ROS2 node interface for map queries
 *
 * This file provides the ROS2 node wrapper for map interface operations:
 * - Subscribes to lane context and route ahead topics
 * - Provides logging through ROS2 node
 * - Loads parameters from params.yaml
 * - Delegates all business logic to MapInterfaceCore
 *
 * WHO WORKS ON THIS:
 * - Sean Yang :)sean.yang@watonomous.ca
 *
 * CURRENT STATUS:
 * - Split into node (this file) and core (map_interface_core.hpp)
 * - Core logic is unit testable without ROS2
 */

#ifndef PREDICTION__MAP_INTERFACE_NODE_HPP_
#define PREDICTION__MAP_INTERFACE_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"
#include "prediction/map_interface_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace prediction {

/**
 * @brief ROS2 node interface to HD map for lanelet queries
 *
 * Wraps MapInterfaceCore with ROS2 subscriptions and logging.
 * For unit testing the core logic, use MapInterfaceCore directly.
 *
 * Parameters (from params.yaml):
 * - max_lanelet_search_depth: Maximum depth for BFS lanelet search (default: 3)
 * - lanelet_cache_size: Maximum number of lanelets to cache (default: 1000)
 */
class MapInterfaceNode {
public:
  /**
   * @brief Construct a new Map Interface Node
   * @param node ROS node pointer for subscriptions, logging, and parameters
   */
  explicit MapInterfaceNode(rclcpp::Node *node);

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
   * Uses max_lanelet_search_depth parameter for depth limit
   * @param current_lanelet_id Current lanelet ID
   * @return Vector of possible future lanelet IDs
   */
  std::vector<int64_t> getPossibleFutureLanelets(int64_t current_lanelet_id);

  /**
   * @brief Get possible future lanelets with custom depth
   * @param current_lanelet_id Current lanelet ID
   * @param max_depth Maximum search depth (overrides parameter)
   * @return Vector of possible future lanelet IDs
   */
  std::vector<int64_t> getPossibleFutureLanelets(int64_t current_lanelet_id,
                                                 int max_depth);

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

  /**
   * @brief Get access to the underlying core for testing
   * @return Reference to the MapInterfaceCore
   */
  MapInterfaceCore &getCore() { return core_; }
  const MapInterfaceCore &getCore() const { return core_; }

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

  rclcpp::Node *node_;
  MapInterfaceCore core_;

  // Parameters
  int max_lanelet_search_depth_;

  // Subscriptions
  rclcpp::Subscription<lanelet_msgs::msg::CurrentLaneContext>::SharedPtr
      lane_context_sub_;
  rclcpp::Subscription<lanelet_msgs::msg::RouteAhead>::SharedPtr
      route_ahead_sub_;

  // Current lane context (for future use)
  lanelet_msgs::msg::CurrentLaneContext::SharedPtr current_lane_context_;
  mutable std::mutex context_mutex_;
};

} // namespace prediction

#endif // PREDICTION__MAP_INTERFACE_NODE_HPP_
