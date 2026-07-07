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

  // Elastic band obstacle avoidance — laterally deforms the path around obstacles before
  // velocity profiling. Disabled by default so existing behaviour is opt-in at the struct
  // level; the node/config yaml turns it on.
  bool elastic_band_enabled{false};
  int eb_max_iterations{50};        // max gradient-descent iterations per deformation
  double eb_step_size{0.2};         // gradient step applied to the combined force each iteration
  double eb_smooth_weight{0.5};     // internal contraction force pulling points onto a smooth curve
  double eb_obstacle_weight{1.5};   // repulsion force pushing points away from nearby obstacles
  double eb_anchor_weight{0.1};     // force pulling points back toward the original path
  double eb_influence_radius{2.5};  // (m) obstacle repulsion falls off to zero beyond this range
  double eb_max_deviation{1.5};     // (m) cap on lateral deviation from the original path
  double eb_convergence_tol{0.01};  // (m) stop iterating once max per-iteration displacement is below this
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
   * @brief Laterally deforms a path around obstacles using a discrete elastic band
   * (Quinlan-Khatib style). The path is assumed to already be expressed in the costmap frame.
   *
   * Each interior point is nudged by a combination of an internal smoothing force (contracts
   * the band onto a smooth curve), an obstacle repulsion force (pushes away from nearby lethal
   * cells), and an anchor force (pulls back toward the original path point) until the band
   * converges or the iteration budget is exhausted. The first and last points never move.
   *
   * @param path The input path to deform.
   * @param costmap The costmap used to locate obstacles.
   * @return nav_msgs::msg::Path The deformed path (same header as the input).
   */
  nav_msgs::msg::Path elastic_band(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & costmap) const;

private:
  int8_t get_max_footprint_cost(double x, double y, double yaw, const nav_msgs::msg::OccupancyGrid & costmap) const;

  TrajectoryConfig config_;
};

}  // namespace trajectory_planner
