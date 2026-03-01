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

#ifndef FREEROAM_PLANNER__ASTAR_HPP_
#define FREEROAM_PLANNER__ASTAR_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace freeroam_planner
{

struct GridCell
{
  int row;
  int col;
};

/// Run A* on an OccupancyGrid from start_world to goal_world (in the grid's frame).
/// Returns a vector of world-frame (x, y) waypoints from start to goal, or empty if no path.
std::vector<std::pair<double, double>> astar(
  const nav_msgs::msg::OccupancyGrid & grid,
  double start_x,
  double start_y,
  double goal_x,
  double goal_y,
  int obstacle_threshold,
  bool allow_diagonal);

}  // namespace freeroam_planner

#endif  // FREEROAM_PLANNER__ASTAR_HPP_
