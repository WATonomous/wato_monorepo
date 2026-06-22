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

#include <cmath>
#include <vector>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "mpc_controller/mpc_core.hpp"

namespace wato
{

using mpc_controller::MpcConfig;
using mpc_controller::MpcCore;
using mpc_controller::StateVec;

// Returns a fully-populated MpcConfig with reasonable test defaults.
static MpcConfig make_test_config()
{
  MpcConfig config{};
  config.lr = 0.0;  // defaults to wheelbase/2 inside the model
  config.horizon_distance = 20.0;
  config.point_spacing = 1.0;
  config.max_horizon_steps = 15;

  config.w_lateral = 50.0;
  config.w_long = 5.0;
  config.w_heading = 20.0;
  config.w_progress = 5.0;
  config.w_speed = 0.0;
  config.w_steering = 1.0;
  config.w_accel = 1.0;
  config.w_dsteering = 100.0;
  config.w_daccel = 50.0;
  config.w_terminal = 5.0;

  config.max_steering_angle = 0.5;
  config.max_accel = 2.5;
  config.max_decel = -4.0;
  config.max_speed = 10.0;

  config.max_steering_rate = 0.3;
  config.max_jerk = 5.0;

  config.dt_max = 0.5;
  config.latency_sec = 0.0;
  config.max_solver_iterations = 200;
  config.solver_eps_abs = 1e-3;
  config.solver_eps_rel = 1e-3;
  config.warm_start = true;

  return config;
}

// Straight trajectory along a given heading (radians), starting at the origin.
static wato_trajectory_msgs::msg::Trajectory make_straight_trajectory_heading(
  double length, double spacing, double speed, double heading)
{
  wato_trajectory_msgs::msg::Trajectory traj;
  for (double s = 0.0; s <= length; s += spacing) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = s * std::cos(heading);
    pt.pose.position.y = s * std::sin(heading);
    pt.pose.orientation.z = std::sin(heading / 2.0);
    pt.pose.orientation.w = std::cos(heading / 2.0);
    pt.max_speed = speed;
    traj.points.push_back(pt);
  }
  return traj;
}

// Constant-curvature arc (circle of radius R) starting at the origin heading +x.
static wato_trajectory_msgs::msg::Trajectory make_arc_trajectory(
  double radius, double arc_length, double spacing, double speed)
{
  wato_trajectory_msgs::msg::Trajectory traj;
  for (double s = 0.0; s <= arc_length; s += spacing) {
    double phi = s / radius;  // heading along the arc
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = radius * std::sin(phi);
    pt.pose.position.y = radius * (1.0 - std::cos(phi));
    pt.pose.orientation.z = std::sin(phi / 2.0);
    pt.pose.orientation.w = std::cos(phi / 2.0);
    pt.max_speed = speed;
    traj.points.push_back(pt);
  }
  return traj;
}

static wato_trajectory_msgs::msg::Trajectory make_straight_trajectory(double length, double spacing, double speed)
{
  wato_trajectory_msgs::msg::Trajectory traj;
  for (double x = 0.0; x <= length; x += spacing) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = x;
    pt.pose.position.y = 0.0;
    pt.pose.orientation.w = 1.0;  // yaw = 0
    pt.max_speed = speed;
    traj.points.push_back(pt);
  }
  return traj;
}

static wato_trajectory_msgs::msg::Trajectory make_stop_trajectory(
  double length, double spacing, double initial_speed, double stop_distance)
{
  wato_trajectory_msgs::msg::Trajectory traj;
  for (double x = 0.0; x <= length; x += spacing) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = x;
    pt.pose.position.y = 0.0;
    pt.pose.orientation.w = 1.0;
    if (x >= stop_distance) {
      pt.max_speed = 0.0;
    } else {
      double ratio = (stop_distance - x) / stop_distance;
      pt.max_speed = initial_speed * ratio;
    }
    traj.points.push_back(pt);
  }
  return traj;
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore sample_reference - basic", "[mpc]")
{
  auto config = make_test_config();
  config.max_horizon_steps = 10;
  config.horizon_distance = 15.0;
  MpcCore mpc(config, 2.5);

  auto traj = make_straight_trajectory(20.0, 0.5, 5.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 5.0;

  auto ref = mpc.sample_reference(traj, state);

  CHECK(ref.size() > 2);
  CHECK(ref.size() <= 11);  // max_horizon_steps + 1

  // All reference points should be along x axis
  for (const auto & rp : ref) {
    CHECK(rp.state(1) == Catch::Approx(0.0).margin(1e-6));
    CHECK(rp.max_speed == Catch::Approx(5.0));
  }
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - straight line tracking", "[mpc]")
{
  auto config = make_test_config();
  MpcCore mpc(config, 2.5);

  auto traj = make_straight_trajectory(30.0, 0.5, 5.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 3.0;  // starting at 3 m/s, target is 5 m/s

  auto ref = mpc.sample_reference(traj, state);
  auto sol = mpc.solve(state, ref, 0.0, 0.0);

  CHECK(sol.solved);
  // Should accelerate (positive acceleration since below speed limit)
  CHECK(sol.acceleration > 0.0);
  // Steering should be near zero on straight line
  CHECK(std::abs(sol.steering_angle) < 0.1);
  // Should have predicted states
  CHECK(sol.predicted_states.size() > 0);
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - speed limit enforcement", "[mpc]")
{
  auto config = make_test_config();
  MpcCore mpc(config, 2.5);

  auto traj = make_straight_trajectory(30.0, 0.5, 3.0);  // speed limit is 3.0
  StateVec state;
  state << 0.0, 0.0, 0.0, 3.0;  // already at speed limit

  auto ref = mpc.sample_reference(traj, state);
  auto sol = mpc.solve(state, ref, 0.0, 0.0);

  CHECK(sol.solved);
  // All predicted velocities should respect the speed limit
  for (const auto & s : sol.predicted_states) {
    CHECK(s(3) <= 3.0 + 0.1);  // small tolerance for solver
  }
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - stop at zero speed", "[mpc]")
{
  auto config = make_test_config();
  config.max_horizon_steps = 20;
  config.horizon_distance = 25.0;
  config.max_decel = -6.0;
  config.max_jerk = 10.0;
  config.max_solver_iterations = 500;
  MpcCore mpc(config, 2.5);

  // Gentle stop: ramp from 5 m/s to 0 over 20m with a long trajectory
  auto traj = make_stop_trajectory(40.0, 0.5, 5.0, 20.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 5.0;

  auto ref = mpc.sample_reference(traj, state);
  auto sol = mpc.solve(state, ref, 0.0, 0.0);

  CHECK(sol.solved);
  // Should be decelerating (negative or near-zero acceleration)
  // The trajectory ramps speed to 0, so MPC should start braking
  CHECK(sol.acceleration < 1.0);
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - heading wrap at +/-pi", "[mpc]")
{
  auto config = make_test_config();
  MpcCore mpc(config, 2.5);

  // Path points straight along -x; its heading is +pi (atan2 returns +pi).
  auto traj = make_straight_trajectory_heading(20.0, 0.5, 5.0, M_PI);

  // Vehicle is aligned with the path but its heading reads -pi (same physical
  // direction, opposite numeric sign). Without unwrapping, the heading cost
  // would see a ~2*pi error and demand a large steering command.
  StateVec state;
  state << 0.0, 0.0, -M_PI, 5.0;

  auto ref = mpc.sample_reference(traj, state);
  REQUIRE(ref.size() >= 2);

  // Unwrapped reference headings must stay close to the vehicle heading.
  for (const auto & rp : ref) {
    CHECK(std::abs(rp.state(2) - (-M_PI)) < 0.2);
  }

  auto sol = mpc.solve(state, ref, 0.0, 0.0);
  CHECK(sol.solved);
  // Aligned with a straight path -> steering must stay near zero.
  CHECK(std::abs(sol.steering_angle) < 0.1);
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - cross-track cost is orientation independent", "[mpc]")
{
  auto config = make_test_config();

  // Case A: straight path along +x, vehicle offset +1 m laterally (in +y).
  MpcCore mpc_a(config, 2.5);
  auto traj_a = make_straight_trajectory_heading(30.0, 0.5, 5.0, 0.0);
  StateVec state_a;
  state_a << 0.0, 1.0, 0.0, 5.0;  // cross-track error = 1.0 m
  auto ref_a = mpc_a.sample_reference(traj_a, state_a);
  auto sol_a = mpc_a.solve(state_a, ref_a, 0.0, 0.0);

  // Case B: identical scenario rotated to a 45-degree diagonal path. The
  // vehicle is offset 1 m perpendicular to the path. With a path-frame cost
  // this must produce essentially the same steering correction as case A;
  // the old global-XY cost would not.
  MpcCore mpc_b(config, 2.5);
  const double h = M_PI / 4.0;
  auto traj_b = make_straight_trajectory_heading(30.0, 0.5, 5.0, h);
  StateVec state_b;
  state_b << -std::sin(h), std::cos(h), h, 5.0;  // 1 m along the path normal
  auto ref_b = mpc_b.sample_reference(traj_b, state_b);
  auto sol_b = mpc_b.solve(state_b, ref_b, 0.0, 0.0);

  REQUIRE(sol_a.solved);
  REQUIRE(sol_b.solved);
  CHECK(sol_a.steering_angle < 0.0);  // steer back toward the path
  CHECK(sol_b.steering_angle < 0.0);
  // Rotation invariance: steering is a frame-independent scalar.
  CHECK(sol_b.steering_angle == Catch::Approx(sol_a.steering_angle).margin(0.02));
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore sample_reference - curvature feedforward", "[mpc]")
{
  auto config = make_test_config();
  config.max_horizon_steps = 15;
  config.horizon_distance = 15.0;
  const double wheelbase = 2.5;
  const double R = 20.0;  // arc radius
  MpcCore mpc(config, wheelbase);

  auto traj = make_arc_trajectory(R, 20.0, 0.5, 5.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 5.0;  // on the arc, aligned

  auto ref = mpc.sample_reference(traj, state);
  REQUIRE(ref.size() > 4);

  const double expected_kappa = 1.0 / R;
  const double expected_delta = std::atan(wheelbase * expected_kappa);

  // Interior reference points carry the curvature feedforward used as the
  // linearization operating point (was always zero before this change).
  CHECK(ref[3].curvature == Catch::Approx(expected_kappa).epsilon(0.1));
  CHECK(ref[3].u_ref(0) == Catch::Approx(expected_delta).epsilon(0.1));
  CHECK(ref[3].u_ref(0) > 0.05);  // clearly nonzero feedforward steering

  auto sol = mpc.solve(state, ref, 0.0, 0.0);
  CHECK(sol.solved);
  CHECK(sol.steering_angle > 0.0);  // turn into the leftward arc
}

TEST_CASE_METHOD(test::TestExecutorFixture, "MpcCore solve - repeated solves stay feasible", "[mpc]")
{
  // Exercises the in-place solver-update path (warm start) across many cycles,
  // including straights where the path-frame Hessian off-diagonal is exactly
  // zero. The sparsity pattern must stay stable (or self-heal via reinit).
  auto config = make_test_config();
  MpcCore mpc(config, 2.5);

  auto traj = make_straight_trajectory(30.0, 0.5, 5.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 4.0;

  double prev_steering = 0.0;
  double prev_accel = 0.0;
  for (int i = 0; i < 6; ++i) {
    auto ref = mpc.sample_reference(traj, state);
    auto sol = mpc.solve(state, ref, prev_steering, prev_accel);
    CHECK(sol.solved);
    prev_steering = sol.steering_angle;
    prev_accel = sol.acceleration;
  }
}

}  // namespace wato
