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

/// Configuration for the elastic lateral deformation planner.
/// Based on Quinlan & Khatib 1993 — adapted for bounded lateral deformation on a costmap.
struct ElasticConfig
{
  // --- Vehicle footprint (body frame, metres) ---
  double footprint_x_min{-0.5};  ///< rear extent
  double footprint_y_min{-1.2};  ///< right side
  double footprint_x_max{3.5};  ///< front bumper
  double footprint_y_max{1.2};  ///< left side
  double footprint_sample_res{0.15};  ///< grid spacing for footprint cost sampling

  // --- Speed ---
  double max_speed{20.0};  ///< default speed cap; overridden by lane limit in node

  // --- Elastic forces ---
  double k_r{0.030};  ///< repulsion gain
  double k_s{0.150};  ///< smoothing gain (Laplacian)
  double max_step{0.050};  ///< max lateral offset change per iteration (Δ, metres)
  double probe_width{0.60};  ///< probe half-width (δ_p, metres)
  int num_iterations{20};  ///< deformation iterations per planning cycle

  // --- Lane corridor ---
  double lane_width{4.0};  ///< full lane width (metres)
  double vehicle_width{2.4};  ///< vehicle width (metres)
  double lane_margin{0.10};  ///< extra safety margin at lane edge (metres)

  // --- Cost gates ---
  double tau_rep{10.0};  ///< repulsion activity threshold: below this C_risk → no repulsion
  double tau_gate{65.0};  ///< lethal zone threshold: reset offset if proposed C_risk >= this
  double epsilon{5.0};  ///< minimum cost improvement over reference to accept a displacement

  // --- Sigmoid speed scaling ---
  double sigmoid_mu{40.0};  ///< sigmoid midpoint cost value (μ)
  double sigmoid_w{20.0};  ///< sigmoid width (w)

  // --- Risk aggregation ---
  double alpha{0.08};  ///< risk sensitivity exponent for log-sum-exp C_risk

  // --- Kinematic backward pass ---
  double a_max{3.0};  ///< max braking deceleration (m/s²) for kinematic backward pass
};

/// Elastic lateral trajectory deformation planner.
///
/// Iteratively deforms a reference path laterally to avoid obstacles using
/// repulsion and elastic smoothing forces, then reconstructs full poses and
/// applies sigmoid-based speed scaling.
class ElasticCore
{
public:
  explicit ElasticCore(const ElasticConfig & config);

  /// Compute elastically deformed, speed-profiled trajectory.
  /// Drop-in compatible with TrajectoryCore::compute_trajectory().
  wato_trajectory_msgs::msg::Trajectory compute_trajectory(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    double limit_speed,
    double current_speed = 0.0);

private:
  /// Stages 0–4: run num_iterations of force application on the offset array d.
  /// d[0] and d[N-1] are anchored to 0 throughout.
  void deform_offsets(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    std::vector<double> & d,
    double d_min,
    double d_max);

  /// Standard sigmoid: σ(x) = 1 / (1 + exp(-x))
  static double sigmoid(double x);

  ElasticConfig config_;
  std::vector<double> cached_offsets_;  ///< warm-start: offset array persisted across ticks
};

}  // namespace trajectory_planner
