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

#include "trajectory_planner/elastic_core.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "trajectory_planner/elastic_costmap.hpp"
#include "trajectory_planner/elastic_forces.hpp"

namespace trajectory_planner
{

ElasticCore::ElasticCore(const ElasticConfig & config)
: config_(config)
{}

double ElasticCore::sigmoid(double x)
{
  return 1.0 / (1.0 + std::exp(-x));
}

wato_trajectory_msgs::msg::Trajectory ElasticCore::compute_trajectory(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double limit_speed,
  [[maybe_unused]] double current_speed)
{
  wato_trajectory_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;

  if (path.poses.empty()) {
    return trajectory;
  }

  const size_t N = path.poses.size();
  double v_max = std::max(0.0, std::min(limit_speed, config_.max_speed));

  // Stage 1 — Lane corridor bounds
  auto [d_min, d_max] = compute_corridor_bounds(config_.lane_width, config_.vehicle_width, config_.lane_margin);

  // Initialise offsets: warm-start from previous tick if path size unchanged
  std::vector<double> d;
  if (cached_offsets_.size() == N) {
    d = cached_offsets_;
  } else {
    d.assign(N, 0.0);
  }

  // Stages 0–4 — Iterative elastic deformation
  deform_offsets(path, costmap, d, d_min, d_max);

  // Save offsets for next tick's warm-start
  cached_offsets_ = d;

  // Stage 5 — Reconstruct deformed positions Q[i] = P[i] + d[i] * n[i]
  std::vector<double> qx(N), qy(N);
  for (size_t i = 0; i < N; ++i) {
    auto tn = compute_tangent_normal(path, i);
    qx[i] = path.poses[i].pose.position.x + d[i] * tn.normal.x;
    qy[i] = path.poses[i].pose.position.y + d[i] * tn.normal.y;
  }

  // Stage 5b — Heading array (central difference over deformed positions)
  std::vector<double> theta_arr(N);
  for (size_t i = 0; i < N; ++i) {
    double dx, dy;
    if (i == 0) {
      dx = qx[1] - qx[0];
      dy = qy[1] - qy[0];
    } else if (i == N - 1) {
      dx = qx[N - 1] - qx[N - 2];
      dy = qy[N - 1] - qy[N - 2];
    } else {
      dx = qx[i + 1] - qx[i - 1];
      dy = qy[i + 1] - qy[i - 1];
    }
    theta_arr[i] = std::atan2(dy, dx);
  }

  // Stage 6 — Two-pass speed profile

  // Pass 1: sigmoid ceiling (C_avg for speed; C_risk for hard stop)
  std::vector<double> v_sigmoid(N);
  for (size_t i = 0; i < N; ++i) {
    auto costs = sample_footprint_costs(
      qx[i],
      qy[i],
      theta_arr[i],
      config_.footprint_x_min,
      config_.footprint_x_max,
      config_.footprint_y_min,
      config_.footprint_y_max,
      config_.footprint_sample_res,
      costmap,
      config_.alpha);

    double x_sig = (costs.avg - config_.sigmoid_mu) / config_.sigmoid_w;
    v_sigmoid[i] = v_max * (1.0 - sigmoid(x_sig));

    if (costs.risk >= config_.tau_gate) {
      v_sigmoid[i] = 0.0;
    }
  }

  // Arc lengths for kinematic pass
  std::vector<double> arc(N, 0.0);
  for (size_t i = 1; i < N; ++i) {
    arc[i] = arc[i - 1] + std::hypot(qx[i] - qx[i - 1], qy[i] - qy[i - 1]);
  }

  // Pass 2: backward kinematic propagation — ensures vehicle can brake in time
  std::vector<double> v_kin = v_sigmoid;
  for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
    double dist = arc[i + 1] - arc[i];
    double v_brake = std::sqrt(v_kin[i + 1] * v_kin[i + 1] + 2.0 * config_.a_max * dist);
    v_kin[i] = std::min(v_kin[i], v_brake);
  }

  // Build trajectory points
  trajectory.points.resize(N);
  for (size_t i = 0; i < N; ++i) {
    wato_trajectory_msgs::msg::TrajectoryPoint & pt = trajectory.points[i];

    pt.pose.position.x = qx[i];
    pt.pose.position.y = qy[i];
    pt.pose.position.z = path.poses[i].pose.position.z;

    // ROS yaw-only quaternion: q = (0, 0, sin(θ/2), cos(θ/2))
    pt.pose.orientation.x = 0.0;
    pt.pose.orientation.y = 0.0;
    pt.pose.orientation.z = std::sin(theta_arr[i] / 2.0);
    pt.pose.orientation.w = std::cos(theta_arr[i] / 2.0);

    pt.max_speed = v_kin[i];
  }

  return trajectory;
}

void ElasticCore::deform_offsets(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  std::vector<double> & d,
  double d_min,
  double d_max)
{
  const size_t N = path.poses.size();
  if (N < 3) {
    return;  // need at least one interior point
  }

  // Pre-pass: test both corridor bounds with C_risk.
  // If both extremes are lethal, swerving cannot help — lock offset to 0.
  std::vector<bool> swerve_possible(N, true);
  for (size_t i = 1; i < N; ++i) {
    auto tn = compute_tangent_normal(path, i);
    double theta = std::atan2(tn.tangent.y, tn.tangent.x);
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;

    double qlx = px + d_max * tn.normal.x;
    double qly = py + d_max * tn.normal.y;
    double qrx = px + d_min * tn.normal.x;
    double qry = py + d_min * tn.normal.y;

    auto cl = sample_footprint_costs(
      qlx,
      qly,
      theta,
      config_.footprint_x_min,
      config_.footprint_x_max,
      config_.footprint_y_min,
      config_.footprint_y_max,
      config_.footprint_sample_res,
      costmap,
      config_.alpha);
    auto cr = sample_footprint_costs(
      qrx,
      qry,
      theta,
      config_.footprint_x_min,
      config_.footprint_x_max,
      config_.footprint_y_min,
      config_.footprint_y_max,
      config_.footprint_sample_res,
      costmap,
      config_.alpha);

    swerve_possible[i] = (cl.risk < config_.tau_gate) || (cr.risk < config_.tau_gate);

    if (!swerve_possible[i]) {
      // Hard-block check: if the reference path itself is also lethal,
      // all points from i onward are unreachable — disable them and stop.
      auto costs_path = sample_footprint_costs(
        px,
        py,
        theta,
        config_.footprint_x_min,
        config_.footprint_x_max,
        config_.footprint_y_min,
        config_.footprint_y_max,
        config_.footprint_sample_res,
        costmap,
        config_.alpha);

      if (costs_path.risk >= config_.tau_gate) {
        for (size_t j = i; j < N; ++j) {
          swerve_possible[j] = false;
          d[j] = 0.0;
        }
        break;
      }
    }
  }

  for (int iter = 0; iter < config_.num_iterations; ++iter) {
    for (size_t i = 1; i < N; ++i) {
      // Skip deformation if swerving cannot help
      if (!swerve_possible[i]) {
        d[i] = 0.0;
        continue;
      }

      auto tn = compute_tangent_normal(path, i);

      // Current deformed position Q[i]
      double qx = path.poses[i].pose.position.x + d[i] * tn.normal.x;
      double qy = path.poses[i].pose.position.y + d[i] * tn.normal.y;

      // Heading approximation: tangent angle (good enough for footprint sampling)
      double theta = std::atan2(tn.tangent.y, tn.tangent.x);

      // Stage 2 — Repulsion force
      double f_rep = compute_repulsion_force(
        qx,
        qy,
        theta,
        tn,
        config_.probe_width,
        config_.k_r,
        config_.tau_rep,
        config_.footprint_x_min,
        config_.footprint_x_max,
        config_.footprint_y_min,
        config_.footprint_y_max,
        config_.footprint_sample_res,
        costmap,
        config_.alpha);

      // Stage 3 — Smoothing force (discrete Laplacian)
      double f_smooth = compute_smoothing_force(d, i, config_.k_s);

      // Stage 4 — Bounded update & validity gate

      // Step 1: combine forces
      double f_total = f_rep + f_smooth;

      // Step 2: clamp step size to prevent overshoot
      double f_clamped = std::clamp(f_total, -config_.max_step, config_.max_step);

      // Step 3: apply and clamp to corridor
      double d_proposed = d[i] + f_clamped;
      double d_corridor = std::clamp(d_proposed, d_min, d_max);

      // Step 4: validity gate — compare proposed vs. reference path cost using C_risk
      double qx_new = path.poses[i].pose.position.x + d_corridor * tn.normal.x;
      double qy_new = path.poses[i].pose.position.y + d_corridor * tn.normal.y;

      auto costs_new = sample_footprint_costs(
        qx_new,
        qy_new,
        theta,
        config_.footprint_x_min,
        config_.footprint_x_max,
        config_.footprint_y_min,
        config_.footprint_y_max,
        config_.footprint_sample_res,
        costmap,
        config_.alpha);

      auto costs_ref = sample_footprint_costs(
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y,
        theta,
        config_.footprint_x_min,
        config_.footprint_x_max,
        config_.footprint_y_min,
        config_.footprint_y_max,
        config_.footprint_sample_res,
        costmap,
        config_.alpha);

      if (costs_new.risk >= config_.tau_gate) {
        // Proposed position is lethal — hard reset to reference path
        d[i] = 0.0;
      } else if (costs_ref.risk - costs_new.risk < config_.epsilon) {
        // Proposed position does not improve enough over reference — reset
        d[i] = 0.0;
      } else {
        // Accept the displacement
        d[i] = d_corridor;
      }
    }
  }
}

}  // namespace trajectory_planner
