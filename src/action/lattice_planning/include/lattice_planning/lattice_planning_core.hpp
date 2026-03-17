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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

struct PathPoint
{
  double x;
  double y;
  double theta;
  double kappa;
};

struct Path
{
  std::vector<PathPoint> path;
  int64_t target_lanelet_id;
  double lateral_dist_from_goal_lane;
  double cost;
};

struct CostFunctionParams
{
  double lateral_movement_weight;
  double physical_limits_weight;
  double preferred_lane_cost;
  double unknown_occupancy_cost;
  double max_curvature_change;
  double centerline_weight;
};

struct PathGenParams
{
  int max_iterations;
  int steps;
  double tolerance;
  double newton_damping;
  double max_step_size;
};

enum class SpiralParam
{
  K0,
  K1,
  K2,
  K3,
  SF
};

enum class SpiralCoeff
{
  C0,
  C1,
  C2,
  C3
};

struct SpiralCoeffConstants
{
  double c1_k0_coeff;
  double c1_k1_coeff;
  double c1_k2_coeff;
  double c1_k3_coeff;
  double c1_divisor;

  double c2_k0_coeff;
  double c2_k1_coeff;
  double c2_k2_coeff;
  double c2_k3_coeff;
  double c2_divisor;

  double c3_k0_coeff;
  double c3_k1_coeff;
  double c3_k2_coeff;
  double c3_k3_coeff;
  double c3_divisor;
};

class LatticePlanningCore
{
public:
  LatticePlanningCore(const SpiralCoeffConstants & constants, const PathGenParams & params);

  // utility functions
  double get_euc_dist(double x1, double y1, double x2, double y2);
  double get_angle_from_pts(double x1, double y1, double x2, double y2);
  double normalise_angle(double angle);

  // path generation
  std::vector<PathPoint> generate_path(PathPoint start, PathPoint target);

  // path costing
  Path get_lowest_cost_path(
    const std::vector<Path> & paths,
    const std::unordered_map<int64_t, int> & preferred_lanelets,
    const CostFunctionParams & cf_params,
    const std::vector<PathPoint> & ego_centerline,
    bool changing_lanes);

  double path_cost_function(
    const Path & path,
    bool preferred_lane,
    const CostFunctionParams params,
    const std::vector<PathPoint> & ego_centerline,
    bool changing_lanes);

  void calculate_spiral_coeff(
    const std::unordered_map<SpiralParam, double> & params, std::unordered_map<SpiralCoeff, double> & coeffs);

  void generate_spiral(
    PathPoint start,
    int steps,
    double sf,
    const std::unordered_map<SpiralCoeff, double> & coeffs,
    std::vector<PathPoint> & path);

  Eigen::Vector3d compute_error_3dof(const PathPoint & actual, const PathPoint & target);

  Eigen::Matrix3d compute_jacobian_3dof(
    const Eigen::Vector3d & p,
    const Eigen::Vector3d & error,
    const PathPoint & start,
    const PathPoint & target,
    int steps);

private:
  SpiralCoeffConstants spiral_constants_;
  PathGenParams pg_params_;
};
