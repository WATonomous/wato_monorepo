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

#include "trajectory_planner/elastic_forces.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "trajectory_planner/elastic_costmap.hpp"

namespace trajectory_planner
{

TangentNormal compute_tangent_normal(const nav_msgs::msg::Path & path, size_t i)
{
  const size_t n = path.poses.size();

  if (n < 2) {
    return TangentNormal{{1.0, 0.0}, {0.0, 1.0}};
  }

  double dx, dy;
  if (i == 0) {
    // Forward difference
    dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
    dy = path.poses[1].pose.position.y - path.poses[0].pose.position.y;
  } else if (i == n - 1) {
    // Backward difference
    dx = path.poses[n - 1].pose.position.x - path.poses[n - 2].pose.position.x;
    dy = path.poses[n - 1].pose.position.y - path.poses[n - 2].pose.position.y;
  } else {
    // Central difference
    dx = path.poses[i + 1].pose.position.x - path.poses[i - 1].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i - 1].pose.position.y;
  }

  double len = std::hypot(dx, dy);
  if (len < 1e-9) {
    return TangentNormal{{1.0, 0.0}, {0.0, 1.0}};
  }

  Vec2 t{dx / len, dy / len};
  Vec2 normal{-t.y, t.x};  // 90° CCW: left = positive
  return TangentNormal{t, normal};
}

std::pair<double, double> compute_corridor_bounds(double lane_width, double vehicle_width, double margin)
{
  double half = lane_width / 2.0 - vehicle_width / 2.0 - margin;
  // Clamp to zero if the lane is too narrow to accommodate the vehicle + margin
  half = std::max(0.0, half);
  return {-half, half};
}

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
  double alpha)
{
  // Activity gate: only apply repulsion near obstacles (using C_risk)
  auto costs_center =
    sample_footprint_costs(cx, cy, theta, fp_x_min, fp_x_max, fp_y_min, fp_y_max, fp_sample_res, costmap, alpha);

  if (costs_center.risk < tau_rep) {
    return 0.0;
  }

  // Left probe: Q_i + probe_width * n
  double lx = cx + probe_width * tn.normal.x;
  double ly = cy + probe_width * tn.normal.y;
  auto costs_left =
    sample_footprint_costs(lx, ly, theta, fp_x_min, fp_x_max, fp_y_min, fp_y_max, fp_sample_res, costmap, alpha);

  // Right probe: Q_i - probe_width * n
  double rx = cx - probe_width * tn.normal.x;
  double ry = cy - probe_width * tn.normal.y;
  auto costs_right =
    sample_footprint_costs(rx, ry, theta, fp_x_min, fp_x_max, fp_y_min, fp_y_max, fp_sample_res, costmap, alpha);

  // F_rep = -k_r * (C_risk_PL - C_risk_PR)
  // Obstacle on left → C_risk_PL > C_risk_PR → F_rep < 0 → push right (correct)
  return -k_r * (costs_left.risk - costs_right.risk);
}

double compute_smoothing_force(const std::vector<double> & d, size_t i, double k_s)
{
  // Start point is a fixed anchor and must not move
  if (i == 0) {
    return 0.0;
  }
  // End point: one-sided backward difference (free boundary)
  if (i == d.size() - 1) {
    return k_s * (d[i - 1] - d[i]);
  }
  // Discrete Laplacian: pulls each point toward the average of its neighbours
  return k_s * (d[i - 1] + d[i + 1] - 2.0 * d[i]);
}

}  // namespace trajectory_planner
