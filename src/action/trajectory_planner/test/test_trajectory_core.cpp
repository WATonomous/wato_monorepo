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

#include <algorithm>
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

// Taller grid variant with more y-range headroom for elastic band deformation tests.
// origin at (-1, -3), resolution 0.5, covers x: -1..19, y: -3..7
static nav_msgs::msg::OccupancyGrid make_tall_empty_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.5;
  grid.info.width = 40;
  grid.info.height = 20;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -3.0;
  grid.data.assign(40 * 20, 0);
  return grid;
}

// Tall grid with a single lethal cell (cost=100) sitting on the path at world position (5, 0).
// cx = (5 - (-1)) / 0.5 = 12,  cy = (0 - (-3)) / 0.5 = 6
static nav_msgs::msg::OccupancyGrid make_tall_grid_with_obstacle_at_5m()
{
  auto grid = make_tall_empty_grid();
  grid.data[6 * 40 + 12] = 100;
  return grid;
}

// Tall grid with a lethal wall spanning the full grid height at world x = 10 — blocks every
// lateral (y) position the elastic band could possibly reach, regardless of deformation.
// cx = (10 - (-1)) / 0.5 = 22
static nav_msgs::msg::OccupancyGrid make_tall_grid_with_full_wall_at_10m()
{
  auto grid = make_tall_empty_grid();
  for (uint32_t cy = 0; cy < grid.info.height; ++cy) {
    grid.data[cy * 40 + 22] = 100;
  }
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
// elastic_band tests
// ---------------------------------------------------------------------------

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band leaves a clear, evenly-spaced path effectively unchanged",
  "[trajectory_core][elastic_band]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.elastic_band_enabled = true;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});
  auto deformed = core.elastic_band(path, make_tall_empty_grid());

  REQUIRE(deformed.poses.size() == path.poses.size());

  double max_disp = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = deformed.poses[i].pose.position.x - path.poses[i].pose.position.x;
    double dy = deformed.poses[i].pose.position.y - path.poses[i].pose.position.y;
    max_disp = std::max(max_disp, std::hypot(dx, dy));
  }

  // With no obstacles, the smoothing force on an already-straight, evenly-spaced path is zero
  // and the anchor force keeps every point pinned to its original position.
  REQUIRE(max_disp < 0.1);
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band deflects the band laterally around a single lethal cell on the path",
  "[trajectory_core][elastic_band]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.elastic_band_enabled = true;
  cfg.interpolation_resolution = 0.1;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});
  auto grid = make_tall_grid_with_obstacle_at_5m();

  // Sanity check: the original, un-deformed path does run through the obstacle.
  REQUIRE(core.find_first_collision(path, grid).has_value());

  auto deformed = core.elastic_band(path, grid);

  // The point sitting on top of the obstacle (index 5, x = 5m) should have moved laterally away
  // from the original path's y = 0 line.
  REQUIRE(std::fabs(deformed.poses[5].pose.position.y) > 0.3);

  // No point should deviate more than the configured cap from the original path.
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = deformed.poses[i].pose.position.x - path.poses[i].pose.position.x;
    double dy = deformed.poses[i].pose.position.y - path.poses[i].pose.position.y;
    REQUIRE(std::hypot(dx, dy) <= cfg.eb_max_deviation + 1e-6);
  }

  // The deformed path should no longer register a lethal collision.
  REQUIRE_FALSE(core.find_first_collision(deformed, grid).has_value());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory still stops for a fully blocked corridor the band cannot clear",
  "[trajectory_core][elastic_band]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.elastic_band_enabled = true;
  cfg.max_speed = 20.0;
  cfg.stop_distance = 0.5;
  cfg.max_tangential_accel = 2.0;
  cfg.max_emergency_accel = 5.0;
  cfg.interpolation_resolution = 0.1;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0});
  auto grid = make_tall_grid_with_full_wall_at_10m();
  auto traj = core.compute_trajectory(path, grid, cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == path.poses.size());

  // The wall spans every lateral position reachable within eb_max_deviation, so the band cannot
  // find a clear deformation; the existing stop-before-collision fallback must still bring the
  // vehicle to a halt at/near the wall.
  REQUIRE(traj.points.back().max_speed == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory leaves path geometry unchanged when elastic_band_enabled is false",
  "[trajectory_core][elastic_band]")
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.elastic_band_enabled = false;
  cfg.max_speed = 20.0;
  cfg.stop_distance = 2.0;
  cfg.max_tangential_accel = 2.0;
  cfg.max_emergency_accel = 5.0;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});
  auto grid = make_tall_grid_with_obstacle_at_5m();
  auto traj = core.compute_trajectory(path, grid, cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    REQUIRE(traj.points[i].pose.position.x == Catch::Approx(path.poses[i].pose.position.x));
    REQUIRE(traj.points[i].pose.position.y == Catch::Approx(path.poses[i].pose.position.y));
  }
}

}  // namespace wato
