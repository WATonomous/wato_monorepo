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

#pragma once

#include <optional>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace trajectory_planner
{

struct TrajectoryConfig
{
  double stop_distance;
  double max_speed;
  double max_tangential_accel;
  double max_emergency_accel;
  double max_lateral_accel;
  double interpolation_resolution;
  double footprint_x_min;  // rear extent (m)
  double footprint_y_min;  // right extent (m)
  double footprint_x_max;  // front extent (m)
  double footprint_y_max;  // left extent (m)

  // A point is only considered for lateral deformation if its footprint cost exceeds this
  // (0-99, below LETHAL_COST). Keeps the vehicle on the centerline when nothing is in the way.
  double deformation_trigger_cost;
  // How far left/right of centre a triggered point is allowed to search for a clear spot (m).
  double max_lateral_shift;
  // Spacing between candidate offsets checked during that search (m). E.g. with
  // max_lateral_shift=0.5 and lateral_search_step=0.1, offsets -0.5, -0.4, ..., 0.5 are tried.
  double lateral_search_step;

  // How many times each point is nudged toward its neighbours' average and toward its target
  // offset before the deformed path is finalized.
  int deformation_iterations;
  // If the largest change across all points in one pass is smaller than this (m), stop early
  // instead of running the remaining iterations.
  double deformation_convergence_tol;
  // How strongly a point is pulled toward the average of its two neighbours each iteration.
  // Higher values produce a smoother curve but take longer to reach the target offset.
  double smoothing_gain;
  // How strongly a point is pulled toward its own target offset (from the search above) each
  // iteration. Higher values reach the target faster but with less smoothing along the way.
  double pull_gain;
  // Cap on how far a point's offset may change in a single iteration (m), regardless of how
  // large smoothing_gain and pull_gain push it. Keeps the loop from overshooting or oscillating.
  double max_step;
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
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    double limit_speed,
    double current_speed = 0.0);

  /**
   * @brief Finds the distance to the first lethal obstacle along the path.
   *
   * @param path The path to check.
   * @param costmap The costmap to check against.
   * @return std::optional<double> Distance in meters, or nullopt if no obstacle.
   */
  std::optional<double> find_first_collision(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap);

  /**
   * @brief Laterally deforms a path around slight costmap obstacles, then relaxes it back
   * toward the centerline. Operates purely on a per-point lateral offset (along the local
   * left-normal), so arc-length spacing between points is preserved.
   *
   * @param path The input path to deform.
   * @param costmap The costmap used to locate obstacles.
   * @return nav_msgs::msg::Path The deformed path (same size/header as the input).
   */
  nav_msgs::msg::Path deform_path(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap);

private:
  int8_t get_max_footprint_cost(double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap) const;

  TrajectoryConfig config_;
};

}  // namespace trajectory_planner
