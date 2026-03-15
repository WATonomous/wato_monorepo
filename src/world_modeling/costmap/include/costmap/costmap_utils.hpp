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

#ifndef COSTMAP__COSTMAP_UTILS_HPP_
#define COSTMAP__COSTMAP_UTILS_HPP_

#include <cmath>
#include <cstdint>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace costmap
{

/**
 * @brief Extract yaw angle from a quaternion.
 */
inline double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/**
 * @brief Stamp a rotated rectangle of @p cost into @p grid.
 */
void markBox(
  nav_msgs::msg::OccupancyGrid & grid, double cx, double cy, double yaw, double half_x, double half_y, int8_t cost);

/**
 * @brief Inflate a single grid cell with a decaying cost kernel.
 */
void inflateCell(nav_msgs::msg::OccupancyGrid & grid, int row, int col, double inflation_m, double cost_decay);

}  // namespace costmap

#endif  // COSTMAP__COSTMAP_UTILS_HPP_
