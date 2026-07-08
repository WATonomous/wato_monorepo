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

  // Naive lateral obstacle avoidance ("elastic" shift). 0.0 disables the feature and
  // reproduces exact longitudinal-only behavior — also the safe default for these
  // otherwise-uninitialized POD fields when a test constructs TrajectoryConfig{} without
  // setting them.
  double max_lateral_shift{0.0};           // max lateral deviation from centerline (m)
  double lateral_search_step{0.1};         // step size for the perpendicular clearance search (m)
  double lateral_clearance_margin{0.3};    // extra buffer beyond the lethal cell (m)
  double lateral_transition_distance{5.0}; // arc length over which the shift ramps in/out (m)
  double lateral_preferred_side_sign{1.0}; // +1 = prefer left on an exact tie, -1 = prefer right
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

private:
  int8_t get_max_footprint_cost(
    double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap,
    double lateral_margin = 0.0) const;

  /**
   * @brief Per-point signed lateral offset (left-positive, relative to path tangent) needed
   * to clear a lethal obstacle; 0.0 where the point is already clear. Contiguous blocked
   * points are resolved to a single avoidance side per run.
   */
  std::vector<double> compute_required_lateral_offsets(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap) const;

  /**
   * @brief Slope-limited dilation of raw_offsets so the shift ramps in/out over
   * config_.lateral_transition_distance without ever undershooting the raw requirement.
   */
  std::vector<double> smooth_lateral_offsets(
    const nav_msgs::msg::Path & path, const std::vector<double> & raw_offsets) const;

  /**
   * @brief Applies smoothed per-point lateral offsets to produce shifted (x, y), and
   * recomputes yaw/orientation from the shifted point sequence itself.
   */
  nav_msgs::msg::Path apply_lateral_offsets(
    const nav_msgs::msg::Path & path, const std::vector<double> & smoothed_offsets) const;

  TrajectoryConfig config_;
};

}  // namespace trajectory_planner
