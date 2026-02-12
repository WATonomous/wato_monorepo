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

#ifndef WATO_TRAJECTORY_PLANNER__TRAJECTORY_CORE_HPP_
#define WATO_TRAJECTORY_PLANNER__TRAJECTORY_CORE_HPP_

#include <optional>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace wato_trajectory_planner
{

struct TrajectoryConfig
{
  double safe_distance = 10.0;
  double stop_distance = 2.0;
  double max_speed = 20.0;  // Default limit if lanelet speed limit unavailable
  double interpolation_resolution = 0.1;
  double footprint_radius = 1.0;  // Simple radius check for now
};

class TrajectoryCore
{
public:
  explicit TrajectoryCore(const TrajectoryConfig & config);

  /**
   * @brief Computes a velocity-profiled trajectory from a path and costmap.
   *
   * @param path The geometric path from local planner.
   * @param costmap The current costmap.
   * @param limit_speed The lane speed limit (m/s). Used with config max_speed to cap velocity.
   * @param current_speed Current vehicle speed (optional, for future smoothing).
   * @return wato_trajectory_msgs::msg::Trajectory
   */
  wato_trajectory_msgs::msg::Trajectory compute_trajectory(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap,
    double limit_speed, double current_speed = 0.0);

  /**
   * @brief Finds the distance to the first lethal obstacle along the path.
   *
   * @param path The path to check.
   * @param costmap The costmap to check against.
   * @return std::optional<double> Distance in meters, or nullopt if no obstacle.
   */
  std::optional<double> find_first_collision(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap);

private:
  TrajectoryConfig config_;
};

}  // namespace wato_trajectory_planner

#endif  // WATO_TRAJECTORY_PLANNER__TRAJECTORY_CORE_HPP_
