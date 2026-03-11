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

#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace trajectory_planner
{

/// 2-D vector used for tangent/normal calculations.
struct Vec2
{
  double x{0.0};
  double y{0.0};
};

/// Unit tangent (direction of travel) and left-pointing unit normal at a path point.
/// Normal is always 90° counter-clockwise from tangent: n = (-t_y, t_x).
/// Left of the vehicle is the +n direction.
struct TangentNormal
{
  Vec2 tangent;  ///< unit vector along path
  Vec2 normal;  ///< unit vector pointing left
};

/// Compute unit tangent and left-pointing normal at index i.
/// Uses central difference for interior points, forward/backward at endpoints.
TangentNormal compute_tangent_normal(const nav_msgs::msg::Path & path, size_t i);

/// Stage 1 — Compute lateral corridor bounds.
/// Returns {d_min, d_max} in metres.
///   d_max =  lane_width/2 - vehicle_width/2 - margin
///   d_min = -d_max
std::pair<double, double> compute_corridor_bounds(double lane_width, double vehicle_width, double margin);

/// Stage 2 — Repulsion force scalar along n_i.
/// Places left/right probes at ±probe_width from (cx, cy) along the normal,
/// samples the vehicle footprint at each probe, and returns:
///   F_rep = -k_r * (C_risk_left_probe - C_risk_right_probe)
/// Returns 0 if C_risk at (cx, cy) < tau_rep (activity gate).
double compute_repulsion_force(
  double cx,
  double cy,
  double theta,
  const TangentNormal & tn,
  double probe_width,
  double k_r,
  double tau_rep,
  double fp_x_min,
  double fp_x_max,
  double fp_y_min,
  double fp_y_max,
  double fp_sample_res,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double alpha);

/// Stage 3 — Elastic smoothing force (discrete Laplacian of d).
///   F_smooth = k_s * (d[i-1] + d[i+1] - 2*d[i])
/// Returns 0 at endpoints (anchors: d[0] = d[N-1] = 0).
double compute_smoothing_force(const std::vector<double> & d, size_t i, double k_s);

}  // namespace trajectory_planner
