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

#include "trajectory_planner/trajectory_core.hpp"

namespace wato
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static nav_msgs::msg::Path make_straight_path(const std::vector<double> & x_positions)
{
  nav_msgs::msg::Path path;
  for (double x : x_positions) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = 0.0;
    path.poses.push_back(ps);
  }
  return path;
}

// All-clear grid, origin at (-1, -1), resolution 0.5, covers x: -1..19, y: -1..1
static nav_msgs::msg::OccupancyGrid make_empty_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.5;
  grid.info.width = 40;
  grid.info.height = 5;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.data.assign(40 * 5, 0);
  return grid;
}

// Same layout with a lethal cell (cost=100) at world position (12, 0).
// cx = (12 - (-1)) / 0.5 = 26,  cy = (0 - (-1)) / 0.5 = 2
static nav_msgs::msg::OccupancyGrid make_grid_with_obstacle_at_12m()
{
  auto grid = make_empty_grid();
  grid.data[2 * 40 + 26] = 100;
  return grid;
}

// Same layout with a single lethal cell (cost=100) at an arbitrary world position,
// for lateral-shift tests where the obstacle needs to be offset from the path centerline.
static nav_msgs::msg::OccupancyGrid make_grid_with_obstacle_at(double x, double y)
{
  auto grid = make_empty_grid();
  int cx = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
  int cy = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);
  grid.data[cy * grid.info.width + cx] = 100;
  return grid;
}

// ---------------------------------------------------------------------------
// Test fixture: TestExecutorFixture initialises rclcpp for us.
// TrajectoryCore is pure computation so we never need to call start_spinning().
// ---------------------------------------------------------------------------

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory returns empty trajectory for empty path",
  "[trajectory_core][speed]")
{
  trajectory_planner::TrajectoryConfig cfg;
  trajectory_planner::TrajectoryCore core(cfg);

  nav_msgs::msg::Path empty_path;
  auto traj = core.compute_trajectory(empty_path, make_empty_grid(), cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.empty());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory assigns full speed when no obstacle present",
  "[trajectory_core][speed]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.max_speed = 20.0;
  cfg.stop_distance = 2.0;
  cfg.max_tangential_accel = 2.0;
  cfg.max_emergency_accel = 5.0;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 4.0, 8.0, 12.0, 16.0});
  auto traj = core.compute_trajectory(path, make_empty_grid(), cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == 5);
  for (const auto & pt : traj.points) {
    REQUIRE(pt.max_speed == Catch::Approx(20.0));
  }
}

// ---------------------------------------------------------------------------
// find_first_collision tests
// ---------------------------------------------------------------------------

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "find_first_collision returns nullopt on clear costmap",
  "[trajectory_core][collision]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 4.0, 8.0, 16.0});
  auto result = core.find_first_collision(path, make_empty_grid());

  REQUIRE_FALSE(result.has_value());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "find_first_collision detects lethal cell and returns approximate distance",
  "[trajectory_core][collision]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.interpolation_resolution = 0.5;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 4.0, 8.0, 12.0, 16.0});
  auto result = core.find_first_collision(path, make_grid_with_obstacle_at_12m());

  REQUIRE(result.has_value());
  REQUIRE(*result == Catch::Approx(12.0).margin(0.6));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "find_first_collision returns nullopt for empty path",
  "[trajectory_core][collision]")
{
  trajectory_planner::TrajectoryConfig cfg;
  trajectory_planner::TrajectoryCore core(cfg);

  nav_msgs::msg::Path empty_path;
  auto result = core.find_first_collision(empty_path, make_empty_grid());

  REQUIRE_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Naive lateral obstacle avoidance tests
//
// Shared scenario: a straight path from x=0..18 (1m spacing, y=0) with a single lethal
// cell at world (10, 0.6) — i.e. offset to the left of centerline, overlapping the vehicle
// footprint (y in [-0.6, 0.6]) only at its very edge. With lateral_clearance_margin=0.2 the
// search footprint half-height becomes 0.8, which is tall enough that shifting left can
// never fully clear the single 0.5m-tall obstacle cell within max_lateral_shift=1.0 (the
// footprint always straddles it), while shifting right clears at exactly 0.4m — making the
// side selection and resulting offsets fully deterministic and hand-verifiable.
// ---------------------------------------------------------------------------

namespace
{
trajectory_planner::TrajectoryConfig make_lateral_test_config()
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.max_speed = 20.0;
  cfg.stop_distance = 2.0;
  cfg.max_tangential_accel = 2.0;
  cfg.max_emergency_accel = 5.0;
  cfg.max_lateral_accel = 50.0;  // large: don't let curvature limiting interfere with this test
  cfg.interpolation_resolution = 0.5;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = -0.6;
  cfg.footprint_y_max = 0.6;
  cfg.max_lateral_shift = 1.0;
  cfg.lateral_search_step = 0.1;
  cfg.lateral_clearance_margin = 0.2;
  cfg.lateral_transition_distance = 4.0;
  cfg.lateral_preferred_side_sign = 1.0;
  return cfg;
}

nav_msgs::msg::Path make_lateral_test_path()
{
  std::vector<double> xs;
  for (double x = 0.0; x <= 18.0; x += 1.0) xs.push_back(x);
  return make_straight_path(xs);
}
}  // namespace

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory shifts laterally around a one-sided obstacle and re-centers after",
  "[trajectory_core][lateral]")
{
  auto cfg = make_lateral_test_config();
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_lateral_test_path();
  auto traj = core.compute_trajectory(path, make_grid_with_obstacle_at(10.0, 0.6), cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == 19);

  // Well before and well after the obstacle, the path stays at the original centerline.
  for (int i = 0; i <= 7; ++i) {
    REQUIRE(traj.points[i].pose.position.y == Catch::Approx(0.0).margin(1e-6));
  }
  for (int i = 12; i <= 18; ++i) {
    REQUIRE(traj.points[i].pose.position.y == Catch::Approx(0.0).margin(1e-6));
  }

  // The shift ramps in, peaks at the obstacle (x=10, index 10), and ramps back out.
  // Negative y = shifted right, away from the obstacle at y=+0.6 (left).
  REQUIRE(traj.points[9].pose.position.y == Catch::Approx(-0.15).margin(0.02));
  REQUIRE(traj.points[10].pose.position.y == Catch::Approx(-0.4).margin(0.02));
  REQUIRE(traj.points[11].pose.position.y == Catch::Approx(-0.15).margin(0.02));

  // Forward progress (x) along the path is unaffected by a purely-lateral shift.
  for (int i = 0; i < 19; ++i) {
    REQUIRE(traj.points[i].pose.position.x == Catch::Approx(static_cast<double>(i)));
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory lateral offsets never exceed the configured ramp slope",
  "[trajectory_core][lateral]")
{
  auto cfg = make_lateral_test_config();
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_lateral_test_path();
  auto traj = core.compute_trajectory(path, make_grid_with_obstacle_at(10.0, 0.6), cfg.max_speed, cfg.max_speed);

  const double slope = cfg.max_lateral_shift / cfg.lateral_transition_distance;
  for (size_t i = 1; i < traj.points.size(); ++i) {
    double dx = traj.points[i].pose.position.x - traj.points[i - 1].pose.position.x;
    double dy = traj.points[i].pose.position.y - traj.points[i - 1].pose.position.y;
    double seg_dist = std::hypot(dx, dy);
    REQUIRE(std::fabs(dy) <= slope * seg_dist + 1e-6);
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "max_lateral_shift of 0.0 disables lateral shifting",
  "[trajectory_core][lateral]")
{
  auto cfg = make_lateral_test_config();
  cfg.max_lateral_shift = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_lateral_test_path();
  auto traj = core.compute_trajectory(path, make_grid_with_obstacle_at(10.0, 0.6), cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == 19);
  for (const auto & pt : traj.points) {
    REQUIRE(pt.pose.position.y == Catch::Approx(0.0).margin(1e-6));
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory falls back to braking when the obstacle cannot be cleared within max_lateral_shift",
  "[trajectory_core][lateral]")
{
  auto cfg = make_lateral_test_config();
  cfg.max_lateral_shift = 0.2;  // less than the 0.4m needed to clear on either side
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_lateral_test_path();
  auto traj = core.compute_trajectory(path, make_grid_with_obstacle_at(10.0, 0.6), cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == 19);
  // The insufficient shift still leaves the footprint overlapping the obstacle, so
  // find_first_collision (re-run on the shifted path) still triggers braking to a stop
  // by the time the vehicle reaches it — no special-case fallback code needed.
  REQUIRE(traj.points[10].max_speed < 1.0);
}

}  // namespace wato
