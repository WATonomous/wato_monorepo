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
  double stop_distance{2.0};
  double max_speed{20.0};
  double max_tangential_accel{2.0};
  double max_emergency_accel{5.0};
  double max_lateral_accel{2.0};
  double interpolation_resolution{0.1};
  double footprint_x_min{-0.5};  // rear extent (m)
  double footprint_y_min{-1.2};  // right extent (m)
  double footprint_x_max{3.5};  // front extent (m)
  double footprint_y_max{1.2};  // left extent (m)

  // Deterministic, costmap-only lateral obstacle avoidance. The node/config yaml enables it.
  bool elastic_band_enabled{false};
  double eb_max_deviation{0.8};
  double eb_lateral_search_step{0.1};
  double eb_clearance_margin{0.3};
  double eb_transition_distance{5.0};
  double eb_preferred_side_sign{1.0};  // +1 is left, -1 is right
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
   * @brief Resamples and laterally deforms a path around lethal costmap obstacles.
   *
   * The path is assumed to be expressed in the costmap frame. A result is returned only when
   * every collision cluster can be cleared and the complete swept path passes final validation.
   * Both endpoint positions remain fixed.
   *
   * @param path The input path to deform.
   * @param costmap The costmap used to locate obstacles.
   * @return A fully valid resampled path, or nullopt if deformation is not possible.
   */
  std::optional<nav_msgs::msg::Path> elastic_band(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap) const;

private:
  nav_msgs::msg::Path resample_path(const nav_msgs::msg::Path & path) const;
  void update_orientations(nav_msgs::msg::Path & path) const;
  int8_t get_max_footprint_cost(
    double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap, double lateral_margin = 0.0) const;
  std::optional<double> find_first_collision_with_margin(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap, double lateral_margin) const;

  TrajectoryConfig config_;
};

}  // namespace trajectory_planner
