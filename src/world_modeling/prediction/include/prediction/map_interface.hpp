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
 * @brief HD MAP QUERY WRAPPER - Provides map information to trajectory generation
 * * WHAT THIS FILE DOES:
 * - Wraps service calls to the lanelet map server
 * - Finds nearest lanelet to object position
 * - Gets lanelet centerlines (path points for trajectory following)
 * - Gets possible future lanelets (for generating multiple hypotheses)
 * - Checks for crosswalks (important for pedestrians and cyclists)
 * - Gets speed limits and regulatory elements
 * * WHO WORKS ON THIS:
 * - Currently using PLACEHOLDERS (no map service required)
 * - When map services available: shared integration task
 * - John needs centerlines for vehicle path following
 * - Girish needs crosswalk detection for pedestrian goals
 * - Aruhant needs crosswalk detection to switch between models
 * * CURRENT STATUS:
 * - Running in PLACEHOLDER MODE
 * - Returns synthetic data for development/testing
 * - Replace with real service calls when world database is ready
 */

#ifndef PREDICTION__MAP_INTERFACE_HPP_
#define PREDICTION__MAP_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace prediction
{

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
class MapInterface
{
public:
  /**
   * @brief Construct a new Map Interface
   * @param node ROS node pointer for service clients
   */
  // FIX: Changed std::shared_ptr to rclcpp::Node*
  explicit MapInterface(rclcpp::Node* node);

  /**
   * @brief Find nearest lanelet to a point
   * @param point Query point
   * @return Lanelet ID of nearest lanelet
   */
  int64_t findNearestLanelet(const geometry_msgs::msg::Point & point);

  /**
   * @brief Get lanelet information by ID
   * @param lanelet_id Lanelet ID
   * @return LaneletInfo structure
   */
  LaneletInfo getLaneletById(int64_t lanelet_id);

  /**
   * @brief Get possible future lanelets from current lanelet
   * @param current_lanelet_id Current lanelet ID
   * @param max_depth Maximum search depth
   * @return Vector of possible future lanelet IDs
   */
  std::vector<int64_t> getPossibleFutureLanelets(
    int64_t current_lanelet_id,
    int max_depth = 3);

  /**
   * @brief Get speed limit for a lanelet
   * @param lanelet_id Lanelet ID
   * @return Speed limit in m/s
   */
  double getSpeedLimit(int64_t lanelet_id);

  /**
   * @brief Check if a crosswalk is near a point
   * @param point Query point
   * @param radius Search radius
   * @return True if crosswalk nearby
   */
  bool isCrosswalkNearby(const geometry_msgs::msg::Point & point, double radius);

private:
  // FIX: Changed std::shared_ptr to rclcpp::Node*
  rclcpp::Node* node_;

  // Service clients for map queries
  // TODO: Add service client declarations
  // rclcpp::Client<wato_msgs::srv::GetLaneletById>::SharedPtr get_lanelet_client_;
  // rclcpp::Client<wato_msgs::srv::FindNearestLanelet>::SharedPtr find_nearest_client_;
  // rclcpp::Client<wato_msgs::srv::ComputeRoute>::SharedPtr compute_route_client_;
  // rclcpp::Client<wato_msgs::srv::GetRegulatoryElements>::SharedPtr get_regulatory_client_;
  // rclcpp::Client<wato_msgs::srv::GetSpeedLimit>::SharedPtr get_speed_limit_client_;
};

}  // namespace prediction

#endif  // PREDICTION__MAP_INTERFACE_HPP_