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

using namespace mpc_controller;

static wato_trajectory_msgs::msg::Trajectory make_straight_trajectory(
  double length, double spacing, double speed)
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
  MpcConfig config;
  config.point_spacing = 1.0;
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
  MpcConfig config;
  config.point_spacing = 1.0;
  config.max_horizon_steps = 15;
  config.horizon_distance = 20.0;
  config.w_lateral = 50.0;
  config.w_heading = 20.0;
  config.w_progress = 5.0;
  config.max_speed = 10.0;
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
  MpcConfig config;
  config.point_spacing = 1.0;
  config.max_horizon_steps = 15;
  config.horizon_distance = 20.0;
  config.max_speed = 10.0;
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
  MpcConfig config;
  config.point_spacing = 1.0;
  config.max_horizon_steps = 20;
  config.horizon_distance = 25.0;
  config.max_speed = 10.0;
  MpcCore mpc(config, 2.5);

  auto traj = make_stop_trajectory(30.0, 0.5, 5.0, 15.0);
  StateVec state;
  state << 0.0, 0.0, 0.0, 5.0;

  auto ref = mpc.sample_reference(traj, state);
  auto sol = mpc.solve(state, ref, 0.0, 0.0);

  CHECK(sol.solved);
  // Should be decelerating (negative or near-zero acceleration)
  // The trajectory ramps speed to 0, so MPC should start braking
  CHECK(sol.acceleration < 1.0);
}

}  // namespace wato
