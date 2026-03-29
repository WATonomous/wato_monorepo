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

#include "mpc_controller/mpc_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mpc_controller
{

static double yaw_from_quaternion(
  double qx, double qy, double qz, double qw)
{
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

static double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

MpcCore::MpcCore(const MpcConfig & config, double wheelbase)
: config_(config), model_(wheelbase)
{}

std::vector<ReferencePoint> MpcCore::sample_reference(
  const wato_trajectory_msgs::msg::Trajectory & trajectory,
  const StateVec & current_state) const
{
  std::vector<ReferencePoint> reference;
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return reference;
  }

  // Find nearest point on trajectory
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    double dx = points[i].pose.position.x - current_state(0);
    double dy = points[i].pose.position.y - current_state(1);
    double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      nearest_idx = i;
    }
  }

  // Walk forward from nearest point, sampling at fixed arc-length spacing
  double accumulated_arc = 0.0;
  double next_sample_arc = 0.0;
  size_t samples = 0;

  for (size_t i = nearest_idx; i < points.size() && samples < static_cast<size_t>(config_.max_horizon_steps + 1); ++i) {
    if (i > nearest_idx) {
      double dx = points[i].pose.position.x - points[i - 1].pose.position.x;
      double dy = points[i].pose.position.y - points[i - 1].pose.position.y;
      accumulated_arc += std::hypot(dx, dy);
    }

    if (accumulated_arc >= next_sample_arc) {
      const auto & pt = points[i];
      const auto & q = pt.pose.orientation;
      double yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);

      ReferencePoint rp;
      rp.state << pt.pose.position.x, pt.pose.position.y, yaw, pt.max_speed;
      rp.max_speed = pt.max_speed;

      // dt = arc_spacing / max(v, v_floor)
      double v_ref = std::max(pt.max_speed, 0.5);
      rp.dt = config_.point_spacing / v_ref;
      rp.dt = std::max(rp.dt, 0.02);  // minimum 20ms step
      rp.dt = std::min(rp.dt, config_.dt_min);

      reference.push_back(rp);
      next_sample_arc += config_.point_spacing;
      samples++;
    }

    if (accumulated_arc >= config_.horizon_distance) break;
  }

  return reference;
}

MpcSolution MpcCore::solve(
  const StateVec & current_state,
  const std::vector<ReferencePoint> & reference,
  double prev_steering,
  double prev_accel)
{
  MpcSolution result;
  if (reference.size() < 2) {
    return result;
  }

  int N = static_cast<int>(reference.size()) - 1;
  N = std::min(N, config_.max_horizon_steps);

  // Linearize model at each reference point
  std::vector<LinearizedModel> models;
  models.reserve(N);
  for (int k = 0; k < N; ++k) {
    ControlVec u_ref = ControlVec::Zero();
    models.push_back(model_.linearize(reference[k].state, u_ref, reference[k].dt));
  }

  return solve_qp(current_state, models, reference, N, prev_steering, prev_accel);
}

MpcSolution MpcCore::solve_qp(
  const StateVec & x0,
  const std::vector<LinearizedModel> & models,
  const std::vector<ReferencePoint> & reference,
  int N,
  double prev_steering,
  double prev_accel)
{
  MpcSolution result;

  // Decision variables: z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
  const int nx = STATE_DIM;
  const int nu = CONTROL_DIM;
  const int n_states = (N + 1) * nx;
  const int n_controls = N * nu;
  const int n_vars = n_states + n_controls;

  // Constraints:
  // 1. Initial state: x_0 = x0 (nx equalities)
  // 2. Dynamics: x_{k+1} = A_k*x_k + B_k*u_k + g_k for k=0..N-1 (N*nx equalities)
  // 3. Speed bounds: 0 <= v_k <= max_speed[k] for k=0..N (N+1 inequalities)
  // 4. Steering bounds: -max <= delta_k <= max for k=0..N-1 (N inequalities)
  // 5. Accel bounds: min <= a_k <= max for k=0..N-1 (N inequalities)
  // 6. Steering rate: -max_rate*dt <= delta_k - delta_{k-1} for k=0..N-1 (N inequalities)
  // 7. Jerk: -max_jerk*dt <= a_k - a_{k-1} for k=0..N-1 (N inequalities)

  const int n_eq = nx + N * nx;                     // initial + dynamics
  const int n_speed = N + 1;                         // speed bounds
  const int n_actuator = N * nu;                     // steering + accel bounds
  const int n_rate = N * nu;                         // steering rate + jerk
  const int n_constraints = n_eq + n_speed + n_actuator + n_rate;

  // Build Hessian P (sparse, upper triangular)
  std::vector<Eigen::Triplet<double>> P_triplets;
  P_triplets.reserve(n_vars);

  // Gradient q
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

  // State cost
  for (int k = 0; k <= N; ++k) {
    int x_offset = k * nx;
    double terminal_mult = (k == N) ? config_.w_terminal : 1.0;

    // Lateral (y) tracking
    P_triplets.emplace_back(x_offset + 1, x_offset + 1, config_.w_lateral * terminal_mult);
    q(x_offset + 1) -= config_.w_lateral * terminal_mult * reference[std::min(k, N)].state(1);

    // Heading tracking
    P_triplets.emplace_back(x_offset + 2, x_offset + 2, config_.w_heading * terminal_mult);
    q(x_offset + 2) -= config_.w_heading * terminal_mult * reference[std::min(k, N)].state(2);

    // Progress: -w_progress * v[k] (linear in q, encourages speed)
    q(x_offset + 3) -= config_.w_progress;

    // Also track x position
    P_triplets.emplace_back(x_offset + 0, x_offset + 0, config_.w_lateral * terminal_mult * 0.1);
    q(x_offset + 0) -= config_.w_lateral * terminal_mult * 0.1 * reference[std::min(k, N)].state(0);
  }

  // Control cost
  for (int k = 0; k < N; ++k) {
    int u_offset = n_states + k * nu;

    // Steering effort
    P_triplets.emplace_back(u_offset + 0, u_offset + 0, config_.w_steering);
    // Acceleration effort
    P_triplets.emplace_back(u_offset + 1, u_offset + 1, config_.w_accel);
  }

  // Control rate cost (smoothness): w * (u_k - u_{k-1})^2
  // Expands to: w * u_k^2 - 2*w*u_k*u_{k-1} + w*u_{k-1}^2
  for (int k = 0; k < N; ++k) {
    int u_offset = n_states + k * nu;

    // Steering rate
    P_triplets.emplace_back(u_offset + 0, u_offset + 0, config_.w_dsteering);
    if (k > 0) {
      int u_prev = n_states + (k - 1) * nu;
      P_triplets.emplace_back(u_prev + 0, u_prev + 0, config_.w_dsteering);
      // Off-diagonal (upper triangular): -w_dsteering
      int row = std::min(u_prev + 0, u_offset + 0);
      int col = std::max(u_prev + 0, u_offset + 0);
      P_triplets.emplace_back(row, col, -config_.w_dsteering);
    } else {
      // k=0: penalize (u_0 - u_prev)^2
      // Only the u_0^2 term (already added above), plus linear term from prev
      q(u_offset + 0) -= config_.w_dsteering * prev_steering;
    }

    // Jerk (acceleration rate)
    P_triplets.emplace_back(u_offset + 1, u_offset + 1, config_.w_daccel);
    if (k > 0) {
      int u_prev = n_states + (k - 1) * nu;
      P_triplets.emplace_back(u_prev + 1, u_prev + 1, config_.w_daccel);
      int row = std::min(u_prev + 1, u_offset + 1);
      int col = std::max(u_prev + 1, u_offset + 1);
      P_triplets.emplace_back(row, col, -config_.w_daccel);
    } else {
      q(u_offset + 1) -= config_.w_daccel * prev_accel;
    }
  }

  // Assemble sparse Hessian (upper triangular)
  Eigen::SparseMatrix<double> P(n_vars, n_vars);
  P.setFromTriplets(P_triplets.begin(), P_triplets.end());
  // OSQP expects the full upper triangular
  P = P.triangularView<Eigen::Upper>();

  // Build constraint matrix A, lower/upper bounds
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.reserve(n_eq * (nx + nu) + n_speed + n_actuator * 2 + n_rate * 2);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(n_constraints);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(n_constraints);

  int row = 0;

  // 1. Initial state constraint: x_0 = x0
  for (int i = 0; i < nx; ++i) {
    A_triplets.emplace_back(row + i, i, 1.0);
    lower(row + i) = x0(i);
    upper(row + i) = x0(i);
  }
  row += nx;

  // 2. Dynamics: x_{k+1} - A_k*x_k - B_k*u_k = g_k
  for (int k = 0; k < N; ++k) {
    const auto & m = models[k];
    int xk = k * nx;
    int xk1 = (k + 1) * nx;
    int uk = n_states + k * nu;

    for (int i = 0; i < nx; ++i) {
      // x_{k+1}
      A_triplets.emplace_back(row + i, xk1 + i, 1.0);

      // -A_k * x_k
      for (int j = 0; j < nx; ++j) {
        if (std::abs(m.A(i, j)) > 1e-12) {
          A_triplets.emplace_back(row + i, xk + j, -m.A(i, j));
        }
      }

      // -B_k * u_k
      for (int j = 0; j < nu; ++j) {
        if (std::abs(m.B(i, j)) > 1e-12) {
          A_triplets.emplace_back(row + i, uk + j, -m.B(i, j));
        }
      }

      lower(row + i) = m.g(i);
      upper(row + i) = m.g(i);
    }
    row += nx;
  }

  // 3. Speed bounds: 0 <= x_k[3] <= max_speed[k]
  for (int k = 0; k <= N; ++k) {
    int v_idx = k * nx + 3;
    A_triplets.emplace_back(row, v_idx, 1.0);
    lower(row) = 0.0;
    upper(row) = std::min(reference[std::min(k, N)].max_speed, config_.max_speed);
    row++;
  }

  // 4. Actuator bounds: steering and acceleration
  for (int k = 0; k < N; ++k) {
    int uk = n_states + k * nu;

    // Steering
    A_triplets.emplace_back(row, uk + 0, 1.0);
    lower(row) = -config_.max_steering_angle;
    upper(row) = config_.max_steering_angle;
    row++;

    // Acceleration
    A_triplets.emplace_back(row, uk + 1, 1.0);
    lower(row) = config_.max_decel;
    upper(row) = config_.max_accel;
    row++;
  }

  // 5. Rate constraints: steering rate and jerk
  for (int k = 0; k < N; ++k) {
    int uk = n_states + k * nu;
    double dt = reference[k].dt;

    if (k > 0) {
      int uk_prev = n_states + (k - 1) * nu;

      // Steering rate: |delta_k - delta_{k-1}| <= max_rate * dt
      A_triplets.emplace_back(row, uk + 0, 1.0);
      A_triplets.emplace_back(row, uk_prev + 0, -1.0);
      lower(row) = -config_.max_steering_rate * dt;
      upper(row) = config_.max_steering_rate * dt;
      row++;

      // Jerk: |a_k - a_{k-1}| <= max_jerk * dt
      A_triplets.emplace_back(row, uk + 1, 1.0);
      A_triplets.emplace_back(row, uk_prev + 1, -1.0);
      lower(row) = -config_.max_jerk * dt;
      upper(row) = config_.max_jerk * dt;
      row++;
    } else {
      // k=0: rate relative to previous applied control
      A_triplets.emplace_back(row, uk + 0, 1.0);
      lower(row) = prev_steering - config_.max_steering_rate * dt;
      upper(row) = prev_steering + config_.max_steering_rate * dt;
      row++;

      A_triplets.emplace_back(row, uk + 1, 1.0);
      lower(row) = prev_accel - config_.max_jerk * dt;
      upper(row) = prev_accel + config_.max_jerk * dt;
      row++;
    }
  }

  // Assemble sparse constraint matrix
  Eigen::SparseMatrix<double> A_mat(n_constraints, n_vars);
  A_mat.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // Solve with OSQP
  bool needs_reinit = !solver_initialized_ || prev_N_ != N;

  if (needs_reinit) {
    solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setMaxIteration(config_.max_solver_iterations);
    solver_->settings()->setAbsoluteTolerance(config_.solver_eps_abs);
    solver_->settings()->setRelativeTolerance(config_.solver_eps_rel);
    solver_->settings()->setWarmStart(config_.warm_start);
    solver_->settings()->setPolish(true);

    solver_->data()->setNumberOfVariables(n_vars);
    solver_->data()->setNumberOfConstraints(n_constraints);
    solver_->data()->setHessianMatrix(P);
    solver_->data()->setGradient(q);
    solver_->data()->setLinearConstraintsMatrix(A_mat);
    solver_->data()->setLowerBound(lower);
    solver_->data()->setUpperBound(upper);

    if (!solver_->initSolver()) {
      return result;
    }
    solver_initialized_ = true;
    prev_N_ = N;
  } else {
    solver_->updateHessianMatrix(P);
    solver_->updateGradient(q);
    solver_->updateLinearConstraintsMatrix(A_mat);
    solver_->updateBounds(lower, upper);

    if (has_prev_solution_ && config_.warm_start) {
      // Shift previous solution forward for warm-start
      Eigen::VectorXd warm_primal = Eigen::VectorXd::Zero(n_vars);

      // Shift states: x_k = prev_x_{k+1}
      for (int k = 0; k < N; ++k) {
        int src = std::min(k + 1, prev_N_) * nx;
        warm_primal.segment(k * nx, nx) = prev_primal_.segment(src, nx);
      }
      warm_primal.segment(N * nx, nx) = warm_primal.segment((N - 1) * nx, nx);

      // Shift controls: u_k = prev_u_{k+1}
      for (int k = 0; k < N - 1; ++k) {
        int src = n_states + std::min(k + 1, prev_N_ - 1) * nu;
        warm_primal.segment(n_states + k * nu, nu) = prev_primal_.segment(src, nu);
      }
      warm_primal.segment(n_states + (N - 1) * nu, nu) =
        warm_primal.segment(n_states + std::max(0, N - 2) * nu, nu);

      solver_->setPrimalVariable(warm_primal);
    }
  }

  auto status = solver_->solveProblem();
  if (status != OsqpEigen::ErrorExitFlag::NoError) {
    return result;
  }

  auto osqp_status = solver_->workspace()->info->status_val;
  if (osqp_status != 1 && osqp_status != 2) {  // OSQP_SOLVED or OSQP_SOLVED_INACCURATE
    return result;
  }

  const Eigen::VectorXd & sol = solver_->getSolution();

  // Store for warm-starting
  prev_primal_ = sol;
  has_prev_solution_ = true;

  // Extract first control
  result.steering_angle = sol(n_states + 0);
  result.acceleration = sol(n_states + 1);
  result.target_speed = sol(3) + result.acceleration * reference[0].dt;
  result.target_speed = std::clamp(result.target_speed, 0.0, config_.max_speed);
  result.solved = true;

  // Extract predicted states for visualization
  result.predicted_states.resize(N + 1);
  for (int k = 0; k <= N; ++k) {
    result.predicted_states[k] = sol.segment(k * nx, nx);
  }

  return result;
}

}  // namespace mpc_controller
