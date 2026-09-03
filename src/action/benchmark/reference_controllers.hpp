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

// Reference-grade comparators: the two controllers the wider field would reach for.
//
//   RegulatedPurePursuit  — the Nav2 nav2_regulated_pure_pursuit_controller algorithm
//                           (Macenski, Singh, Martin, Gines, Autonomous Robots 2023).
//                           Formulas transcribed from regulated_pure_pursuit_controller.cpp
//                           and regulation_functions.hpp.
//
//   ErrorStateMpc         — the Autoware autoware_mpc_lateral_controller formulation:
//                           a path-frame error state [lateral_error, yaw_error, steering],
//                           curvature feedforward, and a first-order steering lag in the
//                           prediction model. Matrices transcribed from
//                           vehicle_model_bicycle_kinematics.cpp.
//
// Both are reimplementations against the same plant and scenarios as the WATonomous
// controllers, so the comparison isolates the control formulation. Gains are the upstream
// defaults where they are dimensionless, and scaled to this vehicle where the upstream
// default targets a small indoor robot (noted inline).

#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace reference_controllers
{

using RefTrajectory = wato_trajectory_msgs::msg::Trajectory;

inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double yaw_of(const RefTrajectory & t, size_t i)
{
  const auto & q = t.points[i].pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ---------------------------------------------------------------------------
// Path geometry: arc length and curvature, precomputed once per scenario.
// ---------------------------------------------------------------------------

struct PathInfo
{
  std::vector<double> s;          // cumulative arc length
  std::vector<double> yaw;        // unwrapped heading
  std::vector<double> curvature;  // dtheta/ds

  static PathInfo build(const RefTrajectory & t)
  {
    PathInfo p;
    const size_t n = t.points.size();
    p.s.resize(n);
    p.yaw.resize(n);
    p.curvature.assign(n, 0.0);

    p.s[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
      p.s[i] = p.s[i - 1] + std::hypot(
                              t.points[i].pose.position.x - t.points[i - 1].pose.position.x,
                              t.points[i].pose.position.y - t.points[i - 1].pose.position.y);
    }

    // unwrap heading so curvature differencing is continuous across +/-pi
    p.yaw[0] = yaw_of(t, 0);
    for (size_t i = 1; i < n; ++i) {
      p.yaw[i] = p.yaw[i - 1] + wrap_pi(yaw_of(t, i) - yaw_of(t, i - 1));
    }

    for (size_t i = 0; i < n; ++i) {
      size_t j = std::min(i + 1, n - 1);
      size_t k = (i == 0) ? 0 : i - 1;
      double ds = p.s[j] - p.s[k];
      p.curvature[i] = (ds > 1e-6) ? (p.yaw[j] - p.yaw[k]) / ds : 0.0;
    }
    return p;
  }
};

// Signed lateral error and heading error against the path (Frenet frame).
struct FrenetState
{
  double lateral_error;
  double yaw_error;
  size_t idx;
};

inline FrenetState to_frenet(
  const RefTrajectory & t, const PathInfo & info, double x, double y, double theta)
{
  double best = 1e18;
  size_t idx = 0;
  for (size_t i = 0; i < t.points.size(); ++i) {
    double d = std::hypot(t.points[i].pose.position.x - x, t.points[i].pose.position.y - y);
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  double ry = info.yaw[idx];
  double dx = x - t.points[idx].pose.position.x;
  double dy = y - t.points[idx].pose.position.y;
  // left-positive lateral offset
  double e_y = -std::sin(ry) * dx + std::cos(ry) * dy;
  double e_yaw = wrap_pi(theta - ry);
  return FrenetState{e_y, e_yaw, idx};
}

// ---------------------------------------------------------------------------
// Nav2 Regulated Pure Pursuit
// ---------------------------------------------------------------------------

struct RppConfig
{
  // Nav2 defaults that are dimensionless / time-based carry over unchanged.
  double lookahead_time = 1.5;  // Nav2 default

  // Nav2's distance defaults (0.3 / 0.9 m) target a sub-metre indoor robot.
  // Scaled to this vehicle; bounds match the WATonomous controller so the
  // comparison isolates the regulation, not the lookahead envelope.
  double min_lookahead_dist = 2.5;
  double max_lookahead_dist = 30.0;

  // Nav2 default 0.9 m, again indoor-robot scale. A passenger car's
  // "sharp turn" threshold is on the order of its minimum turning radius.
  double regulated_linear_scaling_min_radius = 20.0;
  double regulated_linear_scaling_min_speed = 0.5;  // Nav2 default 0.25 m/s

  bool use_regulated_linear_velocity_scaling = true;
  bool use_velocity_scaled_lookahead_dist = true;

  double max_steering_angle = 0.5;  // matched to the vehicle's real limit
  double max_speed = 6.0;
};

struct RppResult
{
  double steering_angle;
  double target_speed;
  double lookahead;
  bool found;
};

// regulation_functions.hpp :: curvatureConstraint
inline double curvature_constraint(double raw_linear_vel, double curvature, double min_radius)
{
  const double radius = std::fabs(1.0 / curvature);
  if (radius < min_radius) {
    return raw_linear_vel * (1.0 - (std::fabs(radius - min_radius) / min_radius));
  }
  return raw_linear_vel;
}

inline RppResult regulated_pure_pursuit(
  const RefTrajectory & traj, double x, double y, double theta, double v,
  double wheelbase, const RppConfig & c)
{
  RppResult r{0.0, 0.0, 0.0, false};

  // getLookAheadDistance
  double lookahead_dist = c.max_lookahead_dist;
  if (c.use_velocity_scaled_lookahead_dist) {
    lookahead_dist = std::fabs(v) * c.lookahead_time;
    lookahead_dist = std::clamp(lookahead_dist, c.min_lookahead_dist, c.max_lookahead_dist);
  }
  r.lookahead = lookahead_dist;

  // carrot: first path point at or beyond the lookahead distance, ahead of the robot
  const double cos_t = std::cos(-theta), sin_t = std::sin(-theta);
  double carrot_x = 0.0, carrot_y = 0.0;
  double raw_speed = c.max_speed;

  for (size_t i = 0; i < traj.points.size(); ++i) {
    double gx = traj.points[i].pose.position.x - x;
    double gy = traj.points[i].pose.position.y - y;
    double bx = gx * cos_t - gy * sin_t;
    double by = gx * sin_t + gy * cos_t;
    double d = std::hypot(bx, by);
    if (bx > 0.0 && (d >= lookahead_dist || i == traj.points.size() - 1)) {
      carrot_x = bx;
      carrot_y = by;
      raw_speed = traj.points[i].max_speed;
      r.found = true;
      break;
    }
  }
  if (!r.found) return r;

  // curvature from the carrot
  const double carrot_dist2 = carrot_x * carrot_x + carrot_y * carrot_y;
  double curvature = (carrot_dist2 > 0.001) ? (2.0 * carrot_y / carrot_dist2) : 0.0;

  // applyConstraints — curvature regulation, then the speed floor
  double linear_vel = raw_speed;
  if (c.use_regulated_linear_velocity_scaling && std::fabs(curvature) > 1e-9) {
    linear_vel = curvature_constraint(raw_speed, curvature, c.regulated_linear_scaling_min_radius);
  }
  // the floor only applies while the path still asks the robot to move
  if (raw_speed > 1e-6) {
    linear_vel = std::max(linear_vel, c.regulated_linear_scaling_min_speed);
  } else {
    linear_vel = 0.0;
  }
  linear_vel = std::min(linear_vel, c.max_speed);

  // Ackermann: angular_vel = v * curvature  =>  steering = atan(L * curvature)
  double steering = std::atan(wheelbase * curvature);
  steering = std::clamp(steering, -c.max_steering_angle, c.max_steering_angle);

  r.steering_angle = steering;
  r.target_speed = linear_vel;
  return r;
}

// ---------------------------------------------------------------------------
// Autoware-style error-state MPC
//
//   state  x = [lateral_error, yaw_error, steering]
//   input  u = steering command
//
//   Ac = [ 0   v                        0      ]
//        [ 0   0    v / (L * cos^2(d_r))       ]
//        [ 0   0                     -1/tau    ]
//   Bc = [ 0, 0, 1/tau ]^T
//   Wc = [ 0, -v*k + (v/L)*(tan(d_r) - d_r/cos^2(d_r)), 0 ]^T
//   with d_r = atan(L*k), discretized bilinear (Tustin).
// ---------------------------------------------------------------------------

struct ErrorMpcConfig
{
  // Autoware defaults
  int prediction_horizon = 50;
  double prediction_dt = 0.1;

  double w_lat_error = 1.0;
  double w_heading_error = 0.0;
  double w_heading_error_squared_vel = 0.3;
  double w_steering_input = 1.0;
  double w_steering_input_squared_vel = 0.25;
  double w_lat_jerk = 0.1;
  double w_steer_rate = 0.0;
  double w_steer_acc = 1e-6;
  double w_terminal_lat_error = 1.0;
  double w_terminal_heading_error = 0.1;

  double steer_tau = 0.3;  // vehicle_model_steer_tau
  double max_steering_angle = 0.5;
  double max_steering_rate = 0.3;

  int max_solver_iterations = 200;
  double solver_eps_abs = 1e-3;
  double solver_eps_rel = 1e-3;
};

struct ErrorMpcSolution
{
  double steering_angle = 0.0;
  bool solved = false;
};

class ErrorStateMpc
{
public:
  ErrorStateMpc(const ErrorMpcConfig & c, double wheelbase)
  : c_(c), wheelbase_(wheelbase)
  {
  }

  // current_steer is the vehicle's actual steering angle (state, not command)
  ErrorMpcSolution solve(
    const RefTrajectory & traj, const PathInfo & info, double x, double y, double theta,
    double v, double current_steer)
  {
    ErrorMpcSolution out;

    const int N = c_.prediction_horizon;
    const double dt = c_.prediction_dt;
    const int nx = 3, nu = 1;

    auto fr = to_frenet(traj, info, x, y, theta);

    // Sample the path forward in time: s advances by v*dt each step.
    // Curvature and velocity are read off the path at each sample.
    std::vector<double> kappa(N + 1), vel(N + 1);
    {
      double s_now = info.s[fr.idx];
      for (int k = 0; k <= N; ++k) {
        double s_k = s_now + v * dt * k;
        // locate s_k on the path
        size_t j = fr.idx;
        while (j + 1 < info.s.size() && info.s[j + 1] < s_k) ++j;
        kappa[k] = info.curvature[j];
        vel[k] = std::max(traj.points[j].max_speed, 0.1);
      }
    }

    const int n_states = (N + 1) * nx;
    const int n_controls = N * nu;
    const int n_vars = n_states + n_controls;

    const int n_eq = nx + N * nx;
    const int n_steer_state = N + 1;  // steering-state bound
    const int n_input = N;            // input bound
    const int n_rate = N;             // input rate bound
    const int n_constraints = n_eq + n_steer_state + n_input + n_rate;

    std::vector<Eigen::Triplet<double>> P_trip;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    for (int k = 0; k <= N; ++k) {
      const int xo = k * nx;
      const bool terminal = (k == N);
      const double vk = vel[k];

      const double w_lat = terminal ? c_.w_terminal_lat_error : c_.w_lat_error;
      const double w_yaw = terminal
                             ? c_.w_terminal_heading_error
                             : (c_.w_heading_error + c_.w_heading_error_squared_vel * vk * vk);

      // 0.5 z'Pz convention: a w*e^2 cost contributes 2w on the diagonal
      P_trip.emplace_back(xo + 0, xo + 0, 2.0 * w_lat);
      P_trip.emplace_back(xo + 1, xo + 1, 2.0 * w_yaw);
    }

    for (int k = 0; k < N; ++k) {
      const int uo = n_states + k * nu;
      const double vk = vel[k];
      const double d_ref = std::atan(wheelbase_ * kappa[k]);

      // steering input error against the curvature feedforward reference
      const double w_in = c_.w_steering_input + c_.w_steering_input_squared_vel * vk * vk;
      P_trip.emplace_back(uo, uo, 2.0 * w_in);
      q(uo) -= 2.0 * w_in * d_ref;

      // lateral jerk ~ (v^2/L) * d(steer)/dt, plus the plain steer-rate term
      const double lat_jerk_gain = (vk * vk / wheelbase_) / dt;
      const double w_rate =
        c_.w_lat_jerk * lat_jerk_gain * lat_jerk_gain + c_.w_steer_rate / (dt * dt);

      // w_rate * (u_k - u_{k-1})^2  ->  diag +2w each, off-diag -2w
      P_trip.emplace_back(uo, uo, 2.0 * w_rate);
      if (k > 0) {
        const int up = n_states + (k - 1) * nu;
        P_trip.emplace_back(up, up, 2.0 * w_rate);
        P_trip.emplace_back(std::min(up, uo), std::max(up, uo), -2.0 * w_rate);
      } else {
        q(uo) -= 2.0 * w_rate * current_steer;
      }

      // steering acceleration, a mild regularizer
      P_trip.emplace_back(uo, uo, 2.0 * c_.w_steer_acc);
    }

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    P.setFromTriplets(P_trip.begin(), P_trip.end());
    Eigen::SparseMatrix<double> P_up = P.triangularView<Eigen::Upper>();

    std::vector<Eigen::Triplet<double>> A_trip;
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(n_constraints);
    int row = 0;

    // initial state
    const double x0[3] = {fr.lateral_error, fr.yaw_error, current_steer};
    for (int i = 0; i < nx; ++i) {
      A_trip.emplace_back(row + i, i, 1.0);
      lower(row + i) = x0[i];
      upper(row + i) = x0[i];
    }
    row += nx;

    // dynamics
    for (int k = 0; k < N; ++k) {
      const double vk = vel[k];
      const double kk = kappa[k];
      const double d_ref = std::atan(wheelbase_ * kk);
      const double cos2 = std::cos(d_ref) * std::cos(d_ref);

      Eigen::Matrix3d Ac = Eigen::Matrix3d::Zero();
      Ac(0, 1) = vk;
      Ac(1, 2) = vk / (wheelbase_ * cos2);
      Ac(2, 2) = -1.0 / c_.steer_tau;

      Eigen::Vector3d Bc(0.0, 0.0, 1.0 / c_.steer_tau);
      Eigen::Vector3d Wc(
        0.0, -vk * kk + (vk / wheelbase_) * (std::tan(d_ref) - d_ref / cos2), 0.0);

      // bilinear (Tustin) discretization, as Autoware does
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d M = (I - dt * 0.5 * Ac).inverse();
      Eigen::Matrix3d Ad = M * (I + dt * 0.5 * Ac);
      Eigen::Vector3d Bd = M * Bc * dt;
      Eigen::Vector3d Wd = M * Wc * dt;

      const int xk = k * nx, xk1 = (k + 1) * nx, uk = n_states + k * nu;
      for (int i = 0; i < nx; ++i) {
        A_trip.emplace_back(row + i, xk1 + i, 1.0);
        for (int j = 0; j < nx; ++j) A_trip.emplace_back(row + i, xk + j, -Ad(i, j));
        A_trip.emplace_back(row + i, uk, -Bd(i));
        lower(row + i) = Wd(i);
        upper(row + i) = Wd(i);
      }
      row += nx;
    }

    // steering state bound
    for (int k = 0; k <= N; ++k) {
      A_trip.emplace_back(row, k * nx + 2, 1.0);
      lower(row) = -c_.max_steering_angle;
      upper(row) = c_.max_steering_angle;
      row++;
    }

    // input bound
    for (int k = 0; k < N; ++k) {
      A_trip.emplace_back(row, n_states + k * nu, 1.0);
      lower(row) = -c_.max_steering_angle;
      upper(row) = c_.max_steering_angle;
      row++;
    }

    // input rate bound, expressed per control step
    for (int k = 0; k < N; ++k) {
      const int uk = n_states + k * nu;
      if (k > 0) {
        A_trip.emplace_back(row, uk, 1.0);
        A_trip.emplace_back(row, n_states + (k - 1) * nu, -1.0);
        lower(row) = -c_.max_steering_rate * dt;
        upper(row) = c_.max_steering_rate * dt;
      } else {
        A_trip.emplace_back(row, uk, 1.0);
        lower(row) = current_steer - c_.max_steering_rate * dt;
        upper(row) = current_steer + c_.max_steering_rate * dt;
      }
      row++;
    }

    Eigen::SparseMatrix<double> A_mat(n_constraints, n_vars);
    A_mat.setFromTriplets(A_trip.begin(), A_trip.end());

    // Keep the QP data alive for as long as the solver holds pointers into it.
    P_ = P_up;
    A_ = A_mat;
    q_ = q;
    l_ = lower;
    u_ = upper;

    if (!solver_ || prev_N_ != N) {
      solver_ = std::make_unique<OsqpEigen::Solver>();
      solver_->settings()->setVerbosity(false);
      solver_->settings()->setMaxIteration(c_.max_solver_iterations);
      solver_->settings()->setAbsoluteTolerance(c_.solver_eps_abs);
      solver_->settings()->setRelativeTolerance(c_.solver_eps_rel);
      solver_->settings()->setWarmStart(true);
      solver_->settings()->setPolish(true);
      solver_->data()->setNumberOfVariables(n_vars);
      solver_->data()->setNumberOfConstraints(n_constraints);
      solver_->data()->setHessianMatrix(P_);
      solver_->data()->setGradient(q_);
      solver_->data()->setLinearConstraintsMatrix(A_);
      solver_->data()->setLowerBound(l_);
      solver_->data()->setUpperBound(u_);
      if (!solver_->initSolver()) {
        solver_.reset();
        return out;
      }
      prev_N_ = N;
    } else {
      solver_->updateHessianMatrix(P_);
      solver_->updateGradient(q_);
      solver_->updateLinearConstraintsMatrix(A_);
      solver_->updateBounds(l_, u_);
    }

    if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      return out;
    }
    const auto st = solver_->workspace()->info->status_val;
    if (st != 1 && st != 2) return out;

    out.steering_angle = solver_->getSolution()(n_states);
    out.solved = true;
    return out;
  }

private:
  ErrorMpcConfig c_;
  double wheelbase_;
  std::unique_ptr<OsqpEigen::Solver> solver_;
  int prev_N_ = -1;

  Eigen::SparseMatrix<double> P_, A_;
  Eigen::VectorXd q_, l_, u_;
};

}  // namespace reference_controllers
