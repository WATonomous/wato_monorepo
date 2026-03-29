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

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#include "mpc_controller/bicycle_model.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace mpc_controller
{

struct MpcConfig
{
  // Horizon
  double horizon_distance = 25.0;
  double point_spacing = 1.0;
  int max_horizon_steps = 30;

  // Cost weights
  double w_lateral = 50.0;
  double w_heading = 20.0;
  double w_progress = 5.0;
  double w_steering = 1.0;
  double w_accel = 1.0;
  double w_dsteering = 100.0;
  double w_daccel = 50.0;
  double w_terminal = 5.0;

  // Actuator limits
  double max_steering_angle = 0.5;    // rad
  double max_accel = 2.5;             // m/s^2
  double max_decel = -4.0;            // m/s^2
  double max_speed = 15.0;            // m/s

  // Rate limits (comfort)
  double max_steering_rate = 0.3;     // rad/s
  double max_jerk = 5.0;             // m/s^3

  // Solver
  double dt_min = 0.5;
  int max_solver_iterations = 200;
  double solver_eps_abs = 1e-3;
  double solver_eps_rel = 1e-3;
  bool warm_start = true;
};

struct ReferencePoint
{
  StateVec state;   // [x, y, theta, v_ref]
  double max_speed;
  double dt;        // time step to next point
};

struct MpcSolution
{
  double steering_angle = 0.0;
  double acceleration = 0.0;
  double target_speed = 0.0;
  bool solved = false;
  std::vector<StateVec> predicted_states;
};

class MpcCore
{
public:
  explicit MpcCore(const MpcConfig & config, double wheelbase);

  /// Sample reference points from trajectory at fixed arc-length spacing.
  /// Returns up to max_horizon_steps reference points starting from nearest point to current_pos.
  std::vector<ReferencePoint> sample_reference(
    const wato_trajectory_msgs::msg::Trajectory & trajectory,
    const StateVec & current_state) const;

  /// Solve MPC given current state and reference trajectory.
  /// prev_steering and prev_accel are used for rate constraint on first step.
  MpcSolution solve(
    const StateVec & current_state,
    const std::vector<ReferencePoint> & reference,
    double prev_steering,
    double prev_accel);

private:
  MpcConfig config_;
  BicycleModel model_;

  std::unique_ptr<OsqpEigen::Solver> solver_;
  bool solver_initialized_ = false;
  int prev_N_ = 0;

  Eigen::VectorXd prev_primal_;
  Eigen::VectorXd prev_dual_;
  bool has_prev_solution_ = false;

  /// Build and solve the QP for the given horizon.
  MpcSolution solve_qp(
    const StateVec & x0,
    const std::vector<LinearizedModel> & models,
    const std::vector<ReferencePoint> & reference,
    int N,
    double prev_steering,
    double prev_accel);
};

}  // namespace mpc_controller
