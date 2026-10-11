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
#include <memory>
#include <vector>

namespace mpc_controller
{

static double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

MpcCore::MpcCore(const MpcConfig & config, double wheelbase)
: config_(config)
, model_(wheelbase, config.lr)
, solver_initialized_(false)
, prev_N_(0)
, has_prev_solution_(false)
{}

std::vector<ReferencePoint> MpcCore::sample_reference(
  const wato_trajectory_msgs::msg::Trajectory & trajectory, const StateVec & current_state) const
{
  std::vector<ReferencePoint> reference;
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return reference;
  }

  // Nearest trajectory point to the vehicle.
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

  // Walk forward from the nearest point, sampling at fixed arc-length spacing.
  double accumulated_arc = 0.0;
  double next_sample_arc = 0.0;
  const size_t max_samples = static_cast<size_t>(config_.max_horizon_steps + 1);

  for (size_t i = nearest_idx; i < points.size() && reference.size() < max_samples; ++i) {
    if (i > nearest_idx) {
      double dx = points[i].pose.position.x - points[i - 1].pose.position.x;
      double dy = points[i].pose.position.y - points[i - 1].pose.position.y;
      accumulated_arc += std::hypot(dx, dy);
    }

    if (accumulated_arc >= next_sample_arc) {
      const auto & pt = points[i];
      const auto & q = pt.pose.orientation;

      ReferencePoint rp;
      rp.state << pt.pose.position.x, pt.pose.position.y,
        yaw_from_quaternion(q.x, q.y, q.z, q.w), pt.max_speed;
      rp.max_speed = pt.max_speed;
      rp.curvature = 0.0;
      rp.u_ref = ControlVec::Zero();

      // Time to traverse one spacing at the reference speed, clamped to a sane range.
      double v_ref = std::max(pt.max_speed, 0.5);
      rp.dt = std::clamp(config_.point_spacing / v_ref, 0.02, config_.dt_max);

      reference.push_back(rp);
      next_sample_arc += config_.point_spacing;
    }

    if (accumulated_arc >= config_.horizon_distance) break;
  }

  if (reference.empty()) {
    return reference;
  }

  // Unwrap reference headings to stay continuous with the vehicle heading; the
  // quadratic heading cost cannot represent the +/-pi branch cut.
  double prev_heading = current_state(2);
  for (auto & rp : reference) {
    rp.state(2) = prev_heading + normalize_angle(rp.state(2) - prev_heading);
    prev_heading = rp.state(2);
  }

  // Per-point curvature and feedforward control (the linearization operating
  // point): steering from path curvature, acceleration from the speed profile.
  const double L = model_.wheelbase();
  const size_t n = reference.size();
  for (size_t k = 0; k < n; ++k) {
    double kappa;
    if (k + 1 < n) {
      kappa = (reference[k + 1].state(2) - reference[k].state(2)) / config_.point_spacing;
    } else {
      kappa = (k > 0) ? reference[k - 1].curvature : 0.0;
    }
    reference[k].curvature = kappa;

    double delta_ref = std::atan(L * kappa);
    delta_ref = std::clamp(delta_ref, -config_.max_steering_angle, config_.max_steering_angle);

    double a_ref = 0.0;
    if (k + 1 < n && reference[k].dt > 1e-6) {
      a_ref = (reference[k + 1].state(3) - reference[k].state(3)) / reference[k].dt;
    }
    a_ref = std::clamp(a_ref, config_.max_decel, config_.max_accel);

    reference[k].u_ref << delta_ref, a_ref;
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

  // Latency compensation: roll the initial state forward under the previous
  // command so the QP plans from where the vehicle will be once this command
  // takes effect. No-op when latency_sec is 0.
  StateVec x0 = current_state;
  if (config_.latency_sec > 1e-6) {
    ControlVec u_prev;
    u_prev << prev_steering, prev_accel;
    x0 = model_.step(x0, u_prev, config_.latency_sec);
  }

  int N = std::min(static_cast<int>(reference.size()) - 1, config_.max_horizon_steps);

  std::vector<LinearizedModel> models;
  models.reserve(N);
  for (int k = 0; k < N; ++k) {
    models.push_back(model_.linearize(reference[k].state, reference[k].u_ref, reference[k].dt));
  }

  return solve_qp(x0, models, reference, N, prev_steering, prev_accel);
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

  // Decision variables: z = [x_0, ..., x_N, u_0, ..., u_{N-1}].
  const int nx = STATE_DIM;
  const int nu = CONTROL_DIM;
  const int n_states = (N + 1) * nx;
  const int n_controls = N * nu;
  const int n_vars = n_states + n_controls;

  // Constraints: initial state + dynamics equalities, then speed bounds
  // (k=1..N), actuator bounds, and rate (steering-rate + jerk) bounds.
  const int n_eq = nx + N * nx;
  const int n_speed = N;
  const int n_actuator = N * nu;
  const int n_rate = N * nu;
  const int n_constraints = n_eq + n_speed + n_actuator + n_rate;

  std::vector<Eigen::Triplet<double>> P_triplets;
  P_triplets.reserve(n_vars);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

  // State cost.
  for (int k = 0; k <= N; ++k) {
    int x_offset = k * nx;
    double terminal_mult = (k == N) ? config_.w_terminal : 1.0;
    const auto & ref = reference[k];

    // Position tracking in the path frame: penalize cross-track and along-track
    // error separately so tracking is independent of the road's world heading.
    //   W = R^T * diag(w_long, w_lateral) * R,  R = rotation by reference heading
    const double th = ref.state(2);
    const double c = std::cos(th);
    const double s = std::sin(th);
    const double wl = config_.w_long * terminal_mult;     // along-track
    const double wt = config_.w_lateral * terminal_mult;  // cross-track
    const double Wxx = wl * c * c + wt * s * s;
    const double Wyy = wl * s * s + wt * c * c;
    const double Wxy = (wl - wt) * c * s;

    // Hessian block (upper triangular). The off-diagonal is always emitted (even
    // when zero on straights) so the sparsity pattern stays fixed across solves.
    P_triplets.emplace_back(x_offset + 0, x_offset + 0, Wxx);
    P_triplets.emplace_back(x_offset + 1, x_offset + 1, Wyy);
    P_triplets.emplace_back(x_offset + 0, x_offset + 1, Wxy);

    q(x_offset + 0) -= Wxx * ref.state(0) + Wxy * ref.state(1);
    q(x_offset + 1) -= Wxy * ref.state(0) + Wyy * ref.state(1);

    // Heading tracking (reference heading is unwrapped, see sample_reference).
    P_triplets.emplace_back(x_offset + 2, x_offset + 2, config_.w_heading * terminal_mult);
    q(x_offset + 2) -= config_.w_heading * terminal_mult * ref.state(2);

    // Progress: linear reward on speed.
    q(x_offset + 3) -= config_.w_progress;

    // Optional quadratic speed tracking toward the reference speed profile.
    if (config_.w_speed > 0.0) {
      P_triplets.emplace_back(x_offset + 3, x_offset + 3, config_.w_speed * terminal_mult);
      q(x_offset + 3) -= config_.w_speed * terminal_mult * ref.state(3);
    }
  }

  // Control effort cost.
  for (int k = 0; k < N; ++k) {
    int u_offset = n_states + k * nu;
    P_triplets.emplace_back(u_offset + 0, u_offset + 0, config_.w_steering);
    P_triplets.emplace_back(u_offset + 1, u_offset + 1, config_.w_accel);
  }

  // Control rate cost (smoothness): w * (u_k - u_{k-1})^2. At k=0 the previous
  // applied command is a constant, contributing only a linear gradient term.
  for (int k = 0; k < N; ++k) {
    int u_offset = n_states + k * nu;

    P_triplets.emplace_back(u_offset + 0, u_offset + 0, config_.w_dsteering);
    P_triplets.emplace_back(u_offset + 1, u_offset + 1, config_.w_daccel);

    if (k > 0) {
      int u_prev = n_states + (k - 1) * nu;
      P_triplets.emplace_back(u_prev + 0, u_prev + 0, config_.w_dsteering);
      P_triplets.emplace_back(u_prev + 0, u_offset + 0, -config_.w_dsteering);
      P_triplets.emplace_back(u_prev + 1, u_prev + 1, config_.w_daccel);
      P_triplets.emplace_back(u_prev + 1, u_offset + 1, -config_.w_daccel);
    } else {
      q(u_offset + 0) -= config_.w_dsteering * prev_steering;
      q(u_offset + 1) -= config_.w_daccel * prev_accel;
    }
  }

  // Assemble the sparse Hessian (OSQP expects the upper triangle).
  Eigen::SparseMatrix<double> P(n_vars, n_vars);
  P.setFromTriplets(P_triplets.begin(), P_triplets.end());
  P = P.triangularView<Eigen::Upper>();

  // Build constraint matrix A and its bounds.
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.reserve(n_eq * (nx + nu) + n_speed + n_actuator * 2 + n_rate * 2);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(n_constraints);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(n_constraints);

  int row = 0;

  // Initial state: x_0 = x0.
  for (int i = 0; i < nx; ++i) {
    A_triplets.emplace_back(row + i, i, 1.0);
    lower(row + i) = x0(i);
    upper(row + i) = x0(i);
  }
  row += nx;

  // Dynamics: x_{k+1} - A_k*x_k - B_k*u_k = g_k. Every A_k/B_k entry is emitted
  // (including zeros) to keep the sparsity pattern fixed for OSQP in-place updates.
  for (int k = 0; k < N; ++k) {
    const auto & m = models[k];
    int xk = k * nx;
    int xk1 = (k + 1) * nx;
    int uk = n_states + k * nu;

    for (int i = 0; i < nx; ++i) {
      A_triplets.emplace_back(row + i, xk1 + i, 1.0);
      for (int j = 0; j < nx; ++j) {
        A_triplets.emplace_back(row + i, xk + j, -m.A(i, j));
      }
      for (int j = 0; j < nu; ++j) {
        A_triplets.emplace_back(row + i, uk + j, -m.B(i, j));
      }
      lower(row + i) = m.g(i);
      upper(row + i) = m.g(i);
    }
    row += nx;
  }

  // Speed bounds: 0 <= v_k <= cap for k=1..N (v_0 is pinned by the initial-state
  // equality). The cap is floored by the lowest speed reachable under the
  // jerk/decel limits from the current speed and command, so entering above the
  // cap stays feasible: the solver brakes at the comfort limit and converges
  // under the cap instead of going infeasible into the open-loop fallback.
  double v_reach = std::max(x0(3), 0.0);
  double a_reach = prev_accel;
  for (int k = 1; k <= N; ++k) {
    double dt = reference[k - 1].dt;
    a_reach = std::max(a_reach - config_.max_jerk * dt, config_.max_decel);
    v_reach = std::max(v_reach + a_reach * dt, 0.0);

    A_triplets.emplace_back(row, k * nx + 3, 1.0);
    lower(row) = 0.0;
    upper(row) = std::max(std::min(reference[k].max_speed, config_.max_speed), v_reach);
    row++;
  }

  // Actuator bounds: steering then acceleration.
  for (int k = 0; k < N; ++k) {
    int uk = n_states + k * nu;

    A_triplets.emplace_back(row, uk + 0, 1.0);
    lower(row) = -config_.max_steering_angle;
    upper(row) = config_.max_steering_angle;
    row++;

    A_triplets.emplace_back(row, uk + 1, 1.0);
    lower(row) = config_.max_decel;
    upper(row) = config_.max_accel;
    row++;
  }

  // Rate bounds: |delta_k - delta_{k-1}| <= max_rate*dt and |a_k - a_{k-1}| <=
  // max_jerk*dt. At k=0 the difference is taken against the previous command,
  // which is a constant and so shifts the bounds instead of adding a column.
  for (int k = 0; k < N; ++k) {
    int uk = n_states + k * nu;
    double dt = reference[k].dt;
    int uk_prev = n_states + (k - 1) * nu;

    A_triplets.emplace_back(row, uk + 0, 1.0);
    if (k > 0) {
      A_triplets.emplace_back(row, uk_prev + 0, -1.0);
      lower(row) = -config_.max_steering_rate * dt;
      upper(row) = config_.max_steering_rate * dt;
    } else {
      lower(row) = prev_steering - config_.max_steering_rate * dt;
      upper(row) = prev_steering + config_.max_steering_rate * dt;
    }
    row++;

    A_triplets.emplace_back(row, uk + 1, 1.0);
    if (k > 0) {
      A_triplets.emplace_back(row, uk_prev + 1, -1.0);
      lower(row) = -config_.max_jerk * dt;
      upper(row) = config_.max_jerk * dt;
    } else {
      lower(row) = prev_accel - config_.max_jerk * dt;
      upper(row) = prev_accel + config_.max_jerk * dt;
    }
    row++;
  }

  Eigen::SparseMatrix<double> A_mat(n_constraints, n_vars);
  A_mat.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // In-place value update when the horizon is unchanged; fall back to a full
  // re-init if any update reports a sparsity mismatch.
  bool needs_reinit = !solver_initialized_ || prev_N_ != N;

  if (!needs_reinit) {
    bool ok = solver_->updateHessianMatrix(P) && solver_->updateGradient(q) &&
              solver_->updateLinearConstraintsMatrix(A_mat) && solver_->updateBounds(lower, upper);

    if (!ok) {
      needs_reinit = true;
    } else if (has_prev_solution_ && config_.warm_start) {
      // Time-shift the previous primal solution forward one step as the guess.
      Eigen::VectorXd warm_primal = Eigen::VectorXd::Zero(n_vars);
      for (int k = 0; k < N; ++k) {
        int src = std::min(k + 1, prev_N_) * nx;
        warm_primal.segment(k * nx, nx) = prev_primal_.segment(src, nx);
      }
      warm_primal.segment(N * nx, nx) = warm_primal.segment((N - 1) * nx, nx);
      for (int k = 0; k < N - 1; ++k) {
        int src = n_states + std::min(k + 1, prev_N_ - 1) * nu;
        warm_primal.segment(n_states + k * nu, nu) = prev_primal_.segment(src, nu);
      }
      warm_primal.segment(n_states + (N - 1) * nu, nu) =
        warm_primal.segment(n_states + std::max(0, N - 2) * nu, nu);

      solver_->setPrimalVariable(warm_primal);
    }
  }

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
  }

  if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    return result;
  }

  int osqp_status = solver_->workspace()->info->status_val;
  if (osqp_status != 1 && osqp_status != 2) {  // not OSQP_SOLVED / OSQP_SOLVED_INACCURATE
    return result;
  }

  const Eigen::VectorXd & sol = solver_->getSolution();
  prev_primal_ = sol;
  has_prev_solution_ = true;

  result.steering_angle = sol(n_states + 0);
  result.acceleration = sol(n_states + 1);
  result.target_speed = std::clamp(sol(nx + 3), 0.0, config_.max_speed);
  result.solved = true;

  result.predicted_states.resize(N + 1);
  for (int k = 0; k <= N; ++k) {
    result.predicted_states[k] = sol.segment(k * nx, nx);
  }

  return result;
}

}  // namespace mpc_controller
