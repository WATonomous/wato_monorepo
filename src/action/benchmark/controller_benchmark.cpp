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

// Closed-loop benchmark comparing the real mpc_controller::MpcCore against the
// adaptive pure pursuit control law, on identical scenarios and an identical
// plant model.
//
// This links the ACTUAL production sources:
//   src/action/mpc_controller/src/mpc_core.cpp
//   src/action/mpc_controller/src/bicycle_model.cpp
//
// The pure pursuit law is transcribed from
//   src/action/ackermann_pure_pursuit/src/pure_pursuit_node.cpp (controlCallback)
// because that logic is welded to the ROS node and cannot be linked standalone.
//
// Configs are the as-deployed values from
//   src/action/action_bringup/config/action.yaml
//
// Emits JSON on stdout.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "mpc_controller/mpc_core.hpp"

using mpc_controller::MpcConfig;
using mpc_controller::MpcCore;
using mpc_controller::StateVec;
using Trajectory = wato_trajectory_msgs::msg::Trajectory;
using TrajectoryPoint = wato_trajectory_msgs::msg::TrajectoryPoint;

// ---------------------------------------------------------------------------
// As-deployed configuration (action_bringup/config/action.yaml)
// ---------------------------------------------------------------------------

static constexpr double WHEELBASE = 2.5667;
static constexpr double CONTROL_RATE_HZ = 20.0;
static constexpr double CONTROL_DT = 1.0 / CONTROL_RATE_HZ;

// Downstream drive-by-wire tracks the commanded speed with a first-order lag.
// Identical for both controllers, so the comparison is fair.
static constexpr double SPEED_LAG_TAU = 0.3;

static MpcConfig deployed_mpc_config()
{
  MpcConfig c{};
  c.horizon_distance = 25.0;
  c.point_spacing = 1.0;
  c.max_horizon_steps = 30;
  c.w_lateral = 50.0;
  c.w_heading = 20.0;
  c.w_progress = 5.0;
  c.w_steering = 1.0;
  c.w_accel = 1.0;
  c.w_dsteering = 100.0;
  c.w_daccel = 50.0;
  c.w_terminal = 5.0;
  c.max_steering_angle = 0.5;
  c.max_accel = 2.5;
  c.max_decel = -4.0;
  c.max_speed = 15.0;
  c.max_steering_rate = 0.3;
  c.max_jerk = 5.0;
  c.dt_min = 0.5;
  c.max_solver_iterations = 200;
  c.solver_eps_abs = 1e-3;
  c.solver_eps_rel = 1e-3;
  c.warm_start = true;
  return c;
}

struct PpConfig
{
  double lookahead_distance = 30.0;
  double min_lookahead_distance = 2.5;
  double lookahead_gain = 2.0;
  double steering_angle_gain = 1.0;
  double max_speed = 6.0;
  double min_speed = 0.0;
  double max_steering_angle = 2.0;  // as deployed: 114 deg, effectively no clamp
  double speed_lookahead_distance = 0.1;
};

// ---------------------------------------------------------------------------
// Plant: nonlinear kinematic bicycle
// ---------------------------------------------------------------------------

struct Pose
{
  double x, y, theta, v;
};

static Pose step_plant(const Pose & s, double steering, double cmd_speed)
{
  Pose n;
  n.x = s.x + s.v * std::cos(s.theta) * CONTROL_DT;
  n.y = s.y + s.v * std::sin(s.theta) * CONTROL_DT;
  n.theta = s.theta + s.v / WHEELBASE * std::tan(steering) * CONTROL_DT;
  n.v = s.v + (cmd_speed - s.v) * (CONTROL_DT / SPEED_LAG_TAU);
  return n;
}

// ---------------------------------------------------------------------------
// Scenario trajectories
// ---------------------------------------------------------------------------

static void push(Trajectory & t, double x, double y, double yaw, double speed)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.max_speed = speed;
  t.points.push_back(p);
}

static Trajectory make_straight(double length, double spacing, double speed)
{
  Trajectory t;
  for (double x = 0.0; x < length; x += spacing) push(t, x, 0.0, 0.0, speed);
  return t;
}

static Trajectory make_curve(double radius, double arc, double spacing, double speed)
{
  Trajectory t;
  for (double s = 0.0; s < arc; s += spacing) {
    double th = s / radius;
    push(t, radius * std::sin(th), radius * (1.0 - std::cos(th)), th, speed);
  }
  return t;
}

static double smoother(double u)
{
  return 6 * u * u * u * u * u - 15 * u * u * u * u + 10 * u * u * u;
}

static Trajectory make_lane_change(double length, double spacing, double speed, double shift)
{
  std::vector<double> xs, ys;
  for (double x = 0.0; x < length; x += spacing) {
    double y;
    if (x < 40.0) {
      y = 0.0;
    } else if (x < 70.0) {
      y = shift * smoother((x - 40.0) / 30.0);
    } else {
      y = shift;
    }
    xs.push_back(x);
    ys.push_back(y);
  }
  Trajectory t;
  for (size_t i = 0; i < xs.size(); ++i) {
    size_t j = std::min(i + 1, xs.size() - 1);
    size_t k = (i == 0) ? 0 : i - 1;
    double yaw = std::atan2(ys[j] - ys[k], xs[j] - xs[k]);
    push(t, xs[i], ys[i], yaw, speed);
  }
  return t;
}

static Trajectory make_double_lane_change(double length, double spacing, double speed, double shift)
{
  std::vector<double> xs, ys;
  for (double x = 0.0; x < length; x += spacing) {
    double y;
    if (x < 40.0) {
      y = 0.0;
    } else if (x < 70.0) {
      y = shift * smoother((x - 40.0) / 30.0);
    } else if (x < 100.0) {
      y = shift;
    } else if (x < 130.0) {
      y = shift * (1.0 - smoother((x - 100.0) / 30.0));
    } else {
      y = 0.0;
    }
    xs.push_back(x);
    ys.push_back(y);
  }
  Trajectory t;
  for (size_t i = 0; i < xs.size(); ++i) {
    size_t j = std::min(i + 1, xs.size() - 1);
    size_t k = (i == 0) ? 0 : i - 1;
    double yaw = std::atan2(ys[j] - ys[k], xs[j] - xs[k]);
    push(t, xs[i], ys[i], yaw, speed);
  }
  return t;
}

static Trajectory make_stop_line(double length, double spacing, double speed, double stop_at)
{
  Trajectory t;
  for (double x = 0.0; x < length; x += spacing) {
    double v = (x >= stop_at) ? 0.0 : speed * (stop_at - x) / stop_at;
    push(t, x, 0.0, 0.0, v);
  }
  return t;
}

// ---------------------------------------------------------------------------
// Pure pursuit — transcribed from pure_pursuit_node.cpp controlCallback
// ---------------------------------------------------------------------------

struct PpResult
{
  double steering_angle;
  double target_speed;
  double lookahead;
  bool found;
};

static PpResult pure_pursuit_control(const Trajectory & traj, const Pose & s, const PpConfig & c)
{
  PpResult r{0.0, 0.0, 0.0, false};

  double adaptive_lookahead =
    std::clamp(c.lookahead_gain * s.v, c.min_lookahead_distance, c.lookahead_distance);
  r.lookahead = adaptive_lookahead;

  double target_speed = c.max_speed;
  double speed_ahead = c.max_speed;
  bool found_speed_point = false;
  double lookahead_x = 0.0, lookahead_y = 0.0;

  const double cos_t = std::cos(-s.theta), sin_t = std::sin(-s.theta);

  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto & pt = traj.points[i];
    double gx = pt.pose.position.x - s.x;
    double gy = pt.pose.position.y - s.y;
    // rotate into base frame
    double dx = gx * cos_t - gy * sin_t;
    double dy = gx * sin_t + gy * cos_t;
    double dist = std::hypot(dx, dy);

    if (!found_speed_point && dx > 0.0 && dist >= c.speed_lookahead_distance) {
      speed_ahead = pt.max_speed;
      found_speed_point = true;
    }

    if (dx > 0.0 && dist >= c.min_lookahead_distance) {
      if (dist >= adaptive_lookahead || i == traj.points.size() - 1) {
        lookahead_x = dx;
        lookahead_y = dy;
        target_speed = speed_ahead;
        r.found = true;
        break;
      }
    }
  }

  if (!r.found) return r;

  double ld_sq = lookahead_x * lookahead_x + lookahead_y * lookahead_y;
  double curvature = 2.0 * lookahead_y / ld_sq;
  double steering = c.steering_angle_gain * std::atan(WHEELBASE * curvature);
  steering = std::clamp(steering, -c.max_steering_angle, c.max_steering_angle);

  double steering_ratio = std::abs(steering) / c.max_steering_angle;
  double speed = target_speed * (1.0 - 0.5 * steering_ratio);
  speed = std::clamp(speed, c.min_speed, c.max_speed);
  if (target_speed <= 0.0) speed = 0.0;

  r.steering_angle = steering;
  r.target_speed = speed;
  return r;
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

struct RunLog
{
  std::vector<double> cte, heading_err, steering, speed, cmd_speed, x, y;
  std::vector<double> solve_ms;
  int failures = 0;
  int no_command = 0;
  double duration = 0.0;
};

static void cross_track(const Trajectory & traj, double x, double y, double & e, size_t & idx)
{
  double best = 1e18;
  idx = 0;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double d = std::hypot(traj.points[i].pose.position.x - x, traj.points[i].pose.position.y - y);
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  size_t j = std::min(idx + 1, traj.points.size() - 1);
  size_t k = (idx == 0) ? 0 : idx - 1;
  double tx = traj.points[j].pose.position.x - traj.points[k].pose.position.x;
  double ty = traj.points[j].pose.position.y - traj.points[k].pose.position.y;
  double n = std::hypot(tx, ty);
  if (n < 1e-9) {
    e = best;
    return;
  }
  double nx = -ty / n, ny = tx / n;
  e = (x - traj.points[idx].pose.position.x) * nx + (y - traj.points[idx].pose.position.y) * ny;
}

static double traj_yaw(const Trajectory & t, size_t i)
{
  const auto & q = t.points[i].pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static double rms(const std::vector<double> & v)
{
  if (v.empty()) return 0.0;
  double s = 0.0;
  for (double x : v) s += x * x;
  return std::sqrt(s / v.size());
}

static double maxabs(const std::vector<double> & v)
{
  double m = 0.0;
  for (double x : v) m = std::max(m, std::abs(x));
  return m;
}

static double percentile(std::vector<double> v, double p)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  double idx = p / 100.0 * (v.size() - 1);
  size_t lo = static_cast<size_t>(idx);
  size_t hi = std::min(lo + 1, v.size() - 1);
  double frac = idx - lo;
  return v[lo] * (1 - frac) + v[hi] * frac;
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

static RunLog simulate_mpc(const Trajectory & traj, double duration)
{
  RunLog log;
  MpcCore mpc(deployed_mpc_config(), WHEELBASE);

  Pose s{traj.points[0].pose.position.x, traj.points[0].pose.position.y, traj_yaw(traj, 0), 0.0};
  double prev_steering = 0.0, prev_accel = 0.0;

  int steps = static_cast<int>(duration / CONTROL_DT);
  for (int n = 0; n < steps; ++n) {
    StateVec state;
    state << s.x, s.y, s.theta, s.v;

    auto t0 = std::chrono::steady_clock::now();
    auto reference = mpc.sample_reference(traj, state);
    double steering, cmd_speed;
    if (reference.size() < 2) {
      log.failures++;
      steering = prev_steering;
      cmd_speed = std::max(0.0, s.v + prev_accel * 0.05);
    } else {
      auto sol = mpc.solve(state, reference, prev_steering, prev_accel);
      if (sol.solved) {
        steering = sol.steering_angle;
        cmd_speed = std::max(0.0, sol.target_speed);
        prev_steering = steering;
        prev_accel = sol.acceleration;
      } else {
        log.failures++;
        steering = prev_steering;
        cmd_speed = std::max(0.0, s.v + prev_accel * 0.05);
      }
    }
    auto t1 = std::chrono::steady_clock::now();
    log.solve_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

    s = step_plant(s, steering, cmd_speed);

    double e;
    size_t idx;
    cross_track(traj, s.x, s.y, e, idx);
    double herr = std::atan2(std::sin(s.theta - traj_yaw(traj, idx)), std::cos(s.theta - traj_yaw(traj, idx)));

    log.cte.push_back(e);
    log.heading_err.push_back(herr);
    log.steering.push_back(steering);
    log.speed.push_back(s.v);
    log.cmd_speed.push_back(cmd_speed);
    log.x.push_back(s.x);
    log.y.push_back(s.y);
    log.duration = (n + 1) * CONTROL_DT;

    if (idx >= traj.points.size() - 3) break;
  }
  return log;
}

static RunLog simulate_pp(const Trajectory & traj, double duration, const PpConfig & cfg)
{
  RunLog log;
  Pose s{traj.points[0].pose.position.x, traj.points[0].pose.position.y, traj_yaw(traj, 0), 0.0};

  int steps = static_cast<int>(duration / CONTROL_DT);
  for (int n = 0; n < steps; ++n) {
    auto t0 = std::chrono::steady_clock::now();
    auto r = pure_pursuit_control(traj, s, cfg);
    auto t1 = std::chrono::steady_clock::now();
    log.solve_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

    if (!r.found) {
      log.no_command++;
      break;
    }

    s = step_plant(s, r.steering_angle, r.target_speed);

    double e;
    size_t idx;
    cross_track(traj, s.x, s.y, e, idx);
    double herr = std::atan2(std::sin(s.theta - traj_yaw(traj, idx)), std::cos(s.theta - traj_yaw(traj, idx)));

    log.cte.push_back(e);
    log.heading_err.push_back(herr);
    log.steering.push_back(r.steering_angle);
    log.speed.push_back(s.v);
    log.cmd_speed.push_back(r.target_speed);
    log.x.push_back(s.x);
    log.y.push_back(s.y);
    log.duration = (n + 1) * CONTROL_DT;

    if (idx >= traj.points.size() - 3) break;
  }
  return log;
}

// ---------------------------------------------------------------------------
// JSON emit
// ---------------------------------------------------------------------------

static void emit_metrics(FILE * out, const std::string & key, const RunLog & log, bool last)
{
  std::vector<double> dsteer;
  for (size_t i = 1; i < log.steering.size(); ++i) {
    dsteer.push_back((log.steering[i] - log.steering[i - 1]) / CONTROL_DT);
  }
  double tv = 0.0;
  for (size_t i = 1; i < log.steering.size(); ++i) tv += std::abs(log.steering[i] - log.steering[i - 1]);

  std::vector<double> lat;
  for (size_t i = 0; i < log.steering.size(); ++i) {
    lat.push_back(log.speed[i] * log.speed[i] * std::tan(std::abs(log.steering[i])) / WHEELBASE);
  }

  double dist = 0.0;
  for (size_t i = 1; i < log.x.size(); ++i) dist += std::hypot(log.x[i] - log.x[i - 1], log.y[i] - log.y[i - 1]);

  double mean_speed = log.speed.empty() ? 0.0
                                        : std::accumulate(log.speed.begin(), log.speed.end(), 0.0) / log.speed.size();

  fprintf(out, "    \"%s\": {\n", key.c_str());
  fprintf(out, "      \"cte_rms\": %.6f,\n", rms(log.cte));
  fprintf(out, "      \"cte_max\": %.6f,\n", maxabs(log.cte));
  fprintf(out, "      \"heading_rms\": %.6f,\n", rms(log.heading_err));
  fprintf(out, "      \"heading_max\": %.6f,\n", maxabs(log.heading_err));
  fprintf(out, "      \"steer_rate_rms\": %.6f,\n", rms(dsteer));
  fprintf(out, "      \"steer_rate_max\": %.6f,\n", maxabs(dsteer));
  fprintf(out, "      \"steer_total_variation\": %.6f,\n", tv);
  fprintf(out, "      \"steer_max\": %.6f,\n", maxabs(log.steering));
  fprintf(out, "      \"lat_accel_max\": %.6f,\n", maxabs(lat));
  fprintf(out, "      \"mean_speed\": %.6f,\n", mean_speed);
  fprintf(out, "      \"final_speed\": %.6f,\n", log.speed.empty() ? 0.0 : log.speed.back());
  fprintf(out, "      \"final_x\": %.6f,\n", log.x.empty() ? 0.0 : log.x.back());
  fprintf(out, "      \"distance\": %.6f,\n", dist);
  fprintf(out, "      \"duration\": %.6f,\n", log.duration);
  fprintf(out, "      \"samples\": %zu,\n", log.cte.size());
  fprintf(out, "      \"failures\": %d,\n", log.failures);
  fprintf(out, "      \"no_command\": %d,\n", log.no_command);
  fprintf(out, "      \"cycle_ms_mean\": %.6f,\n",
         log.solve_ms.empty() ? 0.0
                              : std::accumulate(log.solve_ms.begin(), log.solve_ms.end(), 0.0) / log.solve_ms.size());
  fprintf(out, "      \"cycle_ms_p50\": %.6f,\n", percentile(log.solve_ms, 50));
  fprintf(out, "      \"cycle_ms_p95\": %.6f,\n", percentile(log.solve_ms, 95));
  fprintf(out, "      \"cycle_ms_p99\": %.6f,\n", percentile(log.solve_ms, 99));
  fprintf(out, "      \"cycle_ms_max\": %.6f\n", percentile(log.solve_ms, 100));
  fprintf(out, "    }%s\n", last ? "" : ",");
}

int main(int argc, char ** argv)
{
  std::string only = (argc > 1) ? argv[1] : "";
  std::string outpath = (argc > 2) ? argv[2] : "";

  struct Scenario
  {
    std::string name;
    std::string desc;
    Trajectory traj;
    double duration;
  };

  std::vector<Scenario> scenarios = {
    {"straight", "200 m straight, 6 m/s limit", make_straight(200.0, 0.5, 6.0), 45.0},
    {"curve_r30", "Constant-radius curve, R = 30 m", make_curve(30.0, 140.0, 0.5, 6.0), 45.0},
    {"curve_nowrap", "Curve R = 30 m, heading stays within +/-pi", make_curve(30.0, 85.0, 0.5, 6.0), 30.0},
    {"lane_change", "Single 3.5 m lane change over 30 m", make_lane_change(200.0, 0.5, 6.0, 3.5), 45.0},
    {"double_lane_change", "ISO-style double lane change, 3.5 m",
     make_double_lane_change(220.0, 0.5, 6.0, 3.5), 50.0},
    {"stop_line", "Speed ramp to a full stop at 60 m", make_stop_line(120.0, 0.5, 6.0, 60.0), 40.0},
  };

  PpConfig pp_deployed;
  PpConfig pp_matched;
  pp_matched.max_steering_angle = 0.5;  // matched to MPC actuator limit

  FILE * out = outpath.empty() ? stdout : fopen(outpath.c_str(), "w");
  if (!out) {
    fprintf(stderr, "cannot open %s\n", outpath.c_str());
    return 1;
  }

  std::vector<Scenario *> run;
  for (auto & sc : scenarios) {
    if (only.empty() || sc.name == only) run.push_back(&sc);
  }

  fprintf(out, "{\n  \"scenarios\": {\n");
  for (size_t i = 0; i < run.size(); ++i) {
    auto & sc = *run[i];
    fprintf(out, "    \"%s\": {\n", sc.name.c_str());
    fprintf(out, "      \"desc\": \"%s\",\n", sc.desc.c_str());
    fprintf(out, "      \"results\": {\n");

    auto mpc_log = simulate_mpc(sc.traj, sc.duration);
    auto pp_log = simulate_pp(sc.traj, sc.duration, pp_deployed);
    auto ppm_log = simulate_pp(sc.traj, sc.duration, pp_matched);

    emit_metrics(out, "mpc", mpc_log, false);
    emit_metrics(out, "pure_pursuit_deployed", pp_log, false);
    emit_metrics(out, "pure_pursuit_matched_limits", ppm_log, true);

    fprintf(out, "      }\n");
    fprintf(out, "    }%s\n", i + 1 == run.size() ? "" : ",");
  }
  fprintf(out, "  }\n}\n");
  if (out != stdout) fclose(out);
  return 0;
}
