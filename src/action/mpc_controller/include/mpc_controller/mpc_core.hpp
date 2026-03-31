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

/**
 * @brief All tunable MPC parameters, loaded from ROS params.
 *
 * No defaults here — values are set by the ROS parameter server in
 * MpcControllerNode::on_configure(). See the action YAML config for
 * default values and tuning guidance.
 */
struct MpcConfig
{
  // ---- Horizon ----

  // How far ahead MPC looks (meters)
  double horizon_distance;

  // Arc-length between reference points (meters)
  double point_spacing;

  // Maximum number of QP horizon steps (caps problem size)
  int max_horizon_steps;

  // ---- Cost weights (relative ratios matter, not absolute values) ----

  // Lateral (cross-track) position tracking weight
  double w_lateral;

  // Heading alignment weight
  double w_heading;

  // Speed encouragement weight (linear cost on velocity)
  double w_progress;

  // Steering effort penalty
  double w_steering;

  // Acceleration effort penalty
  double w_accel;

  // Steering rate-of-change penalty (smoothness)
  double w_dsteering;

  // Acceleration rate-of-change penalty (jerk comfort)
  double w_daccel;

  // Multiplier on tracking weights at the final horizon step
  double w_terminal;

  // ---- Actuator limits (hard constraints) ----

  // Maximum steering angle (rad)
  double max_steering_angle;

  // Maximum longitudinal acceleration (m/s^2)
  double max_accel;

  // Maximum deceleration, negative value (m/s^2)
  double max_decel;

  // Maximum longitudinal speed (m/s)
  double max_speed;

  // ---- Rate limits (comfort constraints) ----

  // Maximum steering rate (rad/s)
  double max_steering_rate;

  // Maximum jerk (m/s^3)
  double max_jerk;

  // ---- Solver ----

  // Maximum dt per step when speed is near zero (seconds)
  double dt_min;

  // OSQP iteration cap
  int max_solver_iterations;

  // OSQP absolute convergence tolerance
  double solver_eps_abs;

  // OSQP relative convergence tolerance
  double solver_eps_rel;

  // Reuse previous solution as initial guess
  bool warm_start;
};

/**
 * @brief A single reference point sampled from the trajectory for MPC tracking.
 */
struct ReferencePoint
{
  // Reference state [x, y, theta, v_ref]
  StateVec state;

  // Speed limit at this point (m/s)
  double max_speed;

  // Time step to the next reference point (seconds)
  double dt;
};

/**
 * @brief Result of a single MPC solve iteration.
 */
struct MpcSolution
{
  // Optimal steering command (rad)
  double steering_angle;

  // Optimal acceleration command (m/s^2)
  double acceleration;

  // Target speed after applying acceleration (m/s)
  double target_speed;

  // Whether the QP solver found a valid solution
  bool solved;

  // Predicted state trajectory over the horizon
  std::vector<StateVec> predicted_states;
};

/**
 * @brief Model Predictive Controller using a linearized bicycle model and OSQP.
 *
 * Formulates vehicle trajectory tracking as a quadratic program (QP) at each
 * control cycle. The trajectory from the planner is resampled at fixed arc-length
 * intervals, the bicycle model is linearized at each reference point, and the
 * resulting QP is solved with OSQP via OsqpEigen.
 *
 * Supports warm-starting from the previous solution (time-shifted) for faster
 * convergence. The solver is re-initialized only when the horizon size changes.
 *
 * Not thread-safe — intended to be called from a single timer callback.
 */
class MpcCore
{
public:
  /**
   * @brief Construct the MPC solver with configuration and vehicle parameters.
   * @param config MPC tuning parameters (loaded from ROS params).
   * @param wheelbase Vehicle wheelbase for the bicycle model (meters).
   */
  explicit MpcCore(const MpcConfig & config, double wheelbase);

  /**
   * @brief Sample reference points from a trajectory at fixed arc-length spacing.
   *
   * Finds the nearest trajectory point to current_state, then walks forward
   * sampling at config.point_spacing intervals up to config.horizon_distance
   * or config.max_horizon_steps + 1 points.
   *
   * @param trajectory Input trajectory from the planner.
   * @param current_state Current vehicle state [x, y, theta, v].
   * @return Sampled reference points. Returns fewer than 2 if trajectory is too short.
   */
  std::vector<ReferencePoint> sample_reference(
    const wato_trajectory_msgs::msg::Trajectory & trajectory,
    const StateVec & current_state) const;

  /**
   * @brief Solve MPC for the optimal control action.
   *
   * Linearizes the bicycle model at each reference point, builds the QP,
   * and solves with OSQP. Uses warm-starting when available.
   *
   * @param current_state Current vehicle state [x, y, theta, v].
   * @param reference Reference points from sample_reference().
   * @param prev_steering Previous applied steering angle (for rate constraint at k=0).
   * @param prev_accel Previous applied acceleration (for jerk constraint at k=0).
   * @return MpcSolution with solved=false if the solver fails or reference is too short.
   */
  MpcSolution solve(
    const StateVec & current_state,
    const std::vector<ReferencePoint> & reference,
    double prev_steering,
    double prev_accel);

private:
  // Tuning parameters
  MpcConfig config_;

  // Kinematic bicycle model for linearization
  BicycleModel model_;

  // ---- OSQP solver state ----
  std::unique_ptr<OsqpEigen::Solver> solver_;

  // Whether the solver has been initialized with a QP
  bool solver_initialized_;

  // Previous horizon length (triggers re-init on change)
  int prev_N_;

  // ---- Warm-start state ----

  // Previous primal solution for warm-starting
  Eigen::VectorXd prev_primal_;

  // Previous dual solution (reserved for future use)
  Eigen::VectorXd prev_dual_;

  // Whether a previous solution is available
  bool has_prev_solution_;

  /**
   * @brief Build and solve the QP for a given horizon.
   *
   * Constructs the Hessian (P), gradient (q), constraint matrix (A), and
   * bounds from the linearized models and reference. Handles solver
   * initialization, warm-starting, and solution extraction.
   *
   * @param x0 Initial state constraint.
   * @param models Linearized bicycle models at each horizon step.
   * @param reference Reference points (N+1 entries for N steps).
   * @param N Horizon length (number of control steps).
   * @param prev_steering Previous steering for rate constraint at k=0.
   * @param prev_accel Previous acceleration for jerk constraint at k=0.
   * @return MpcSolution with the first control input and predicted trajectory.
   */
  MpcSolution solve_qp(
    const StateVec & x0,
    const std::vector<LinearizedModel> & models,
    const std::vector<ReferencePoint> & reference,
    int N,
    double prev_steering,
    double prev_accel);
};

}  // namespace mpc_controller
