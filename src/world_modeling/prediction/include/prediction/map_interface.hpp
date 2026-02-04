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
 * @brief DEPRECATED - Use world_model::LaneletHandler instead.
 *
 * This interface is obsolete. Lanelet information should be obtained from
 * world_model::LaneletHandler or by subscribing to lanelet_ahead/route_ahead topics.
 *
 * @see world_model/lanelet_handler.hpp
 */

#ifndef PREDICTION__MAP_INTERFACE_HPP_
#define PREDICTION__MAP_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

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
 * @brief DEPRECATED - Interface to HD map for lanelet queries.
 *
 * This class is obsolete. Use Lanelet messages directly instead.
 */
class MapInterface
{
public:
  /**
   * @brief Construct a new Map Interface
   * @param node ROS node pointer for service clients
   */
  explicit MapInterface(rclcpp::Node * node);

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
  std::vector<int64_t> getPossibleFutureLanelets(int64_t current_lanelet_id, int max_depth = 3);

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
  rclcpp::Node * node_;
};

}  // namespace prediction

#endif  // PREDICTION__MAP_INTERFACE_HPP_
