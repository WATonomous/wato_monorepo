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

#include "lattice_planning/lattice_planning_core.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

static double perp_distance(const PathPoint & pt, const std::vector<PathPoint> & centerline)
{
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const auto & a = centerline[i];
    const auto & b = centerline[i + 1];

    double dx = b.x - a.x, dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    double t = len_sq > 1e-9 ? std::clamp(((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len_sq, 0.0, 1.0) : 0.0;

    double px = a.x + t * dx - pt.x;
    double py = a.y + t * dy - pt.y;
    min_dist = std::min(min_dist, std::sqrt(px * px + py * py));
  }

  return (min_dist == std::numeric_limits<double>::max()) ? 0.0 : min_dist;
}

LatticePlanningCore::LatticePlanningCore(const SpiralCoeffConstants & constants, const PathGenParams & params)
: spiral_constants_(constants)
, pg_params_(params)
{}

/**
 * @brief Calculate Euclidean distance between two 2D points.
 */
double LatticePlanningCore::get_euc_dist(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Calculate angle from first point to second point.
 */
double LatticePlanningCore::get_angle_from_pts(double x1, double y1, double x2, double y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

/**
 * @brief Normalize angle to [-pi, pi].
 */
double LatticePlanningCore::normalise_angle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

/**
 * @brief Calculate cubic spiral polynomial coefficients from boundary conditions.
 *
 * Computes coefficients for k(s) = c0 + c1*s + c2*s^2 + c3*s^3 using a cubic interpolation scheme
 * that ensures smooth curvature transitions between waypoints.
 */
void LatticePlanningCore::calculate_spiral_coeff(
  const std::unordered_map<SpiralParam, double> & params, std::unordered_map<SpiralCoeff, double> & coeffs)
{
  const double k0 = params.at(SpiralParam::K0);
  const double k1 = params.at(SpiralParam::K1);
  const double k2 = params.at(SpiralParam::K2);
  const double k3 = params.at(SpiralParam::K3);
  const double sf = params.at(SpiralParam::SF);

  coeffs[SpiralCoeff::C0] = k0;

  coeffs[SpiralCoeff::C1] = (spiral_constants_.c1_k0_coeff * k0 + spiral_constants_.c1_k1_coeff * k1 +
                             spiral_constants_.c1_k2_coeff * k2 + spiral_constants_.c1_k3_coeff * k3) /
                            (spiral_constants_.c1_divisor * sf);

  coeffs[SpiralCoeff::C2] = (spiral_constants_.c2_k0_coeff * k0 + spiral_constants_.c2_k1_coeff * k1 +
                             spiral_constants_.c2_k2_coeff * k2 + spiral_constants_.c2_k3_coeff * k3) /
                            (spiral_constants_.c2_divisor * sf * sf);

  coeffs[SpiralCoeff::C3] = (spiral_constants_.c3_k0_coeff * k0 + spiral_constants_.c3_k1_coeff * k1 +
                             spiral_constants_.c3_k2_coeff * k2 + spiral_constants_.c3_k3_coeff * k3) /
                            (spiral_constants_.c3_divisor * sf * sf * sf);
}

/**
 * @brief Select path with lowest cost from candidate set.
 */
Path LatticePlanningCore::get_lowest_cost_path(
  const std::vector<Path> & paths,
  const std::unordered_map<int64_t, int> & preferred_lanelets,
  const CostFunctionParams & cf_params,
  const std::vector<PathPoint> & ego_centerline,
  bool changing_lanes)
{
  Path lowest_cost_path = paths[0];
  double prev_cost = path_cost_function(
    paths[0], preferred_lanelets.count(paths[0].target_lanelet_id) >= 1, cf_params, ego_centerline, changing_lanes);

  for (size_t i = 1; i < paths.size(); i++) {
    bool preferred_lane = preferred_lanelets.count(paths[i].target_lanelet_id) >= 1;
    double path_cost = path_cost_function(paths[i], preferred_lane, cf_params, ego_centerline, changing_lanes);

    if (path_cost < prev_cost) {
      lowest_cost_path = paths[i];
      prev_cost = path_cost;
    }
  }
  return lowest_cost_path;
}

/**
 * @brief Compute cost of a path based on curvature changes, lateral movement, lane preference,
 *        and centerline deviation when not changing lanes.
 */
double LatticePlanningCore::path_cost_function(
  const Path & path,
  bool preferred_lane,
  const CostFunctionParams params,
  const std::vector<PathPoint> & ego_centerline,
  bool changing_lanes)
{
  double path_cost = 0.0;
  double prev_kappa = std::numeric_limits<double>::quiet_NaN();

  if (!preferred_lane) {
    path_cost += params.preferred_lane_cost;
  }

  for (const auto & pt : path.path) {
    if (!std::isnan(prev_kappa)) {
      double curvature_change = fabs(pt.kappa - prev_kappa);
      if (curvature_change > params.max_curvature_change) {
        path_cost += params.physical_limits_weight * (curvature_change - params.max_curvature_change);
      }
    }
    path_cost += params.lateral_movement_weight * fabs(pt.kappa);
    prev_kappa = pt.kappa;
  }

  if (!changing_lanes && !ego_centerline.empty()) {
    double total = 0.0;
    for (const auto & pt : path.path) {
      total += perp_distance(pt, ego_centerline);
    }
    path_cost += params.centerline_weight * (total / path.path.size());
  }

  return path_cost;
}

/**
 * @brief Generate discrete path points along a cubic spiral by integrating curvature.
 */
void LatticePlanningCore::generate_spiral(
  PathPoint start,
  int steps,
  double sf,
  const std::unordered_map<SpiralCoeff, double> & coeffs,
  std::vector<PathPoint> & path)
{
  const double c0 = coeffs.at(SpiralCoeff::C0);
  const double c1 = coeffs.at(SpiralCoeff::C1);
  const double c2 = coeffs.at(SpiralCoeff::C2);
  const double c3 = coeffs.at(SpiralCoeff::C3);

  double ds = sf / steps;
  double x = start.x, y = start.y, theta = start.theta;

  path.push_back(start);

  for (int i = 1; i <= steps; ++i) {
    double s = i * ds;
    double kappa = c0 + c1 * s + c2 * s * s + c3 * s * s * s;
    theta += kappa * ds;
    x += cos(theta) * ds;
    y += sin(theta) * ds;
    path.push_back({x, y, theta, kappa});
  }
}

/**
 * @brief Compute 3-DOF error vector (x, y, theta) between actual and target path points.
 */
Eigen::Vector3d LatticePlanningCore::compute_error_3dof(const PathPoint & actual, const PathPoint & target)
{
  Eigen::Vector3d error;
  error << actual.x - target.x, actual.y - target.y, normalise_angle(actual.theta - target.theta);
  return error;
}

/**
 * @brief Generate smooth path between start and target using damped Newton optimization.
 */
std::vector<PathPoint> LatticePlanningCore::generate_path(PathPoint start, PathPoint target)
{
  std::vector<PathPoint> path;
  std::unordered_map<SpiralCoeff, double> spiral_coeffs;
  std::unordered_map<SpiralParam, double> spiral_params;

  double norm = std::numeric_limits<double>::infinity();
  double sf_est = get_euc_dist(start.x, start.y, target.x, target.y);

  spiral_params[SpiralParam::K0] = start.kappa;
  spiral_params[SpiralParam::K1] = start.kappa + (target.kappa - start.kappa) * (1.0 / 3.0);
  spiral_params[SpiralParam::K2] = start.kappa + (target.kappa - start.kappa) * (2.0 / 3.0);
  spiral_params[SpiralParam::K3] = target.kappa;
  spiral_params[SpiralParam::SF] = sf_est;

  double max_kappa = std::max({std::abs(start.kappa), std::abs(target.kappa), 0.5});

  for (int i = 0; i < pg_params_.max_iterations; i++) {
    path.clear();

    calculate_spiral_coeff(spiral_params, spiral_coeffs);
    generate_spiral(start, pg_params_.steps, spiral_params[SpiralParam::SF], spiral_coeffs, path);

    PathPoint final_state = path.back();
    Eigen::Vector3d error = compute_error_3dof(final_state, target);
    norm = error.norm();

    if (norm < pg_params_.tolerance) break;

    Eigen::Vector3d params(
      spiral_params[SpiralParam::K1], spiral_params[SpiralParam::K2], spiral_params[SpiralParam::SF]);
    Eigen::Matrix3d J = compute_jacobian_3dof(params, error, start, target, pg_params_.steps);

    double det = J.determinant();
    if (std::abs(det) < 1e-10) {
      std::cerr << "Singular Jacobian (det=" << det << ") at iter " << i << "\n";
      path.clear();
      return path;
    }

    Eigen::Vector3d delta = J.inverse() * error;
    delta *= pg_params_.newton_damping;

    if (delta.norm() > pg_params_.max_step_size) {
      delta = delta.normalized() * pg_params_.max_step_size;
    }

    params -= delta;
    spiral_params[SpiralParam::K1] = std::clamp(params(0), -max_kappa, max_kappa);
    spiral_params[SpiralParam::K2] = std::clamp(params(1), -max_kappa, max_kappa);
    spiral_params[SpiralParam::SF] = std::max(0.5, params(2));
  }

  if (norm > pg_params_.tolerance) {
    std::cerr << "Failed to converge. Final error: " << norm << "\n";
    path.clear();
  }

  return path;
}

/**
 * @brief Compute 3x3 Jacobian matrix d(x,y,theta)/d(k1,k2,sf) using finite differences.
 */
Eigen::Matrix3d LatticePlanningCore::compute_jacobian_3dof(
  const Eigen::Vector3d & p,
  const Eigen::Vector3d & error,
  const PathPoint & start,
  const PathPoint & target,
  int steps)
{
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  const double delta = 1e-6;

  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d p_perturbed = p;
    p_perturbed(j) += delta;

    std::vector<PathPoint> temp_path;
    std::unordered_map<SpiralCoeff, double> spiral_coeffs;
    std::unordered_map<SpiralParam, double> spiral_params;

    spiral_params[SpiralParam::K0] = start.kappa;
    spiral_params[SpiralParam::K1] = p_perturbed(0);
    spiral_params[SpiralParam::K2] = p_perturbed(1);
    spiral_params[SpiralParam::K3] = target.kappa;
    spiral_params[SpiralParam::SF] = p_perturbed(2);

    calculate_spiral_coeff(spiral_params, spiral_coeffs);
    generate_spiral(start, steps, p_perturbed(2), spiral_coeffs, temp_path);

    PathPoint final_state = temp_path.back();
    Eigen::Vector3d error_perturbed = compute_error_3dof(final_state, target);

    jacobian.col(j) = (error_perturbed - error) / delta;
  }

  return jacobian;
}
