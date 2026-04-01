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

}  // namespace wato
