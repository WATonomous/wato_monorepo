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
#include <limits>
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

static nav_msgs::msg::OccupancyGrid make_fine_empty_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.1;
  grid.info.width = 250;
  grid.info.height = 80;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -4.05;
  grid.data.assign(grid.info.width * grid.info.height, 0);
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

static void mark_lethal_cell(nav_msgs::msg::OccupancyGrid & grid, double x, double y)
{
  const int col = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
  const int row = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
  REQUIRE(col >= 0);
  REQUIRE(row >= 0);
  REQUIRE(col < static_cast<int>(grid.info.width));
  REQUIRE(row < static_cast<int>(grid.info.height));
  grid.data[static_cast<size_t>(row) * grid.info.width + static_cast<size_t>(col)] = 100;
}

static trajectory_planner::TrajectoryConfig make_elastic_config()
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.stop_distance = 0.5;
  cfg.max_speed = 10.0;
  cfg.max_tangential_accel = 2.0;
  cfg.max_emergency_accel = 5.0;
  cfg.max_lateral_accel = 50.0;
  cfg.interpolation_resolution = 0.25;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = -0.5;
  cfg.footprint_y_max = 0.5;
  cfg.elastic_band_enabled = true;
  cfg.eb_max_deviation = 0.8;
  cfg.eb_lateral_search_step = 0.1;
  cfg.eb_clearance_margin = 0.2;
  cfg.eb_transition_distance = 2.0;
  cfg.eb_preferred_side_sign = 1.0;
  return cfg;
}

static size_t nearest_x_index(const nav_msgs::msg::Path & path, double x)
{
  size_t best = 0;
  double best_error = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double error = std::fabs(path.poses[i].pose.position.x - x);
    if (error < best_error) {
      best = i;
      best_error = error;
    }
  }
  return best;
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
// deterministic elastic-band tests
// ---------------------------------------------------------------------------

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band resamples a sparse clear path and preserves its endpoints",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);

  const auto path = make_straight_path({0.0, 10.0, 20.0});
  const auto result = core.elastic_band(path, make_tall_empty_grid());

  REQUIRE(result.has_value());
  REQUIRE(result->poses.size() == 81);
  REQUIRE(result->poses.front().pose.position.x == Catch::Approx(0.0));
  REQUIRE(result->poses.back().pose.position.x == Catch::Approx(20.0));
  for (const auto & pose : result->poses) {
    REQUIRE(pose.pose.position.y == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(pose.pose.orientation.z == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(pose.pose.orientation.w == Catch::Approx(1.0).margin(1e-9));
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band finds an obstacle between sparse waypoints and shifts to the smaller clear side",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 10.0, 0.5);  // overlaps the left edge of the footprint

  const auto result = core.elastic_band(make_straight_path({0.0, 20.0}), grid);

  REQUIRE(result.has_value());
  const size_t obstacle_index = nearest_x_index(*result, 10.0);
  REQUIRE(result->poses[obstacle_index].pose.position.y < -0.1);
  REQUIRE(result->poses.front().pose.position.y == Catch::Approx(0.0).margin(1e-9));
  REQUIRE(result->poses.back().pose.position.y == Catch::Approx(0.0).margin(1e-9));

  double max_deviation = 0.0;
  for (const auto & pose : result->poses) {
    max_deviation = std::max(max_deviation, std::fabs(pose.pose.position.y));
  }
  REQUIRE(max_deviation <= cfg.eb_max_deviation + 1e-9);
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band uses the configured left tie break for a symmetric obstacle",
  "[trajectory_core][elastic_band]")
{
  auto cfg = make_elastic_config();
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.eb_clearance_margin = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  // Center the path vertically in a costmap cell so occupancy quantization does not break the
  // intended left/right tie.
  grid.info.origin.position.y = -3.25;
  mark_lethal_cell(grid, 10.0, 0.0);

  const auto result = core.elastic_band(make_straight_path({0.0, 20.0}), grid);

  REQUIRE(result.has_value());
  REQUIRE(result->poses[nearest_x_index(*result, 10.0)].pose.position.y > 0.0);
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band rejects an obstacle too close to preserve the anchored transition",
  "[trajectory_core][elastic_band]")
{
  auto cfg = make_elastic_config();
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.eb_clearance_margin = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 1.0, 0.0);

  REQUIRE_FALSE(core.elastic_band(make_straight_path({0.0, 20.0}), grid).has_value());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band can use the maximum offset and still returns smoothly to anchored endpoints",
  "[trajectory_core][elastic_band]")
{
  auto cfg = make_elastic_config();
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.eb_clearance_margin = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_fine_empty_grid();
  for (int step = -7; step <= 7; ++step) {
    mark_lethal_cell(grid, 10.0, step * 0.1);
  }

  const auto result = core.elastic_band(make_straight_path({0.0, 20.0}), grid);

  REQUIRE(result.has_value());
  REQUIRE(
    result->poses[nearest_x_index(*result, 10.0)].pose.position.y == Catch::Approx(cfg.eb_max_deviation).margin(1e-9));
  REQUIRE(result->poses.front().pose.position.y == Catch::Approx(0.0).margin(1e-9));
  REQUIRE(result->poses.back().pose.position.y == Catch::Approx(0.0).margin(1e-9));
  REQUIRE(result->poses[nearest_x_index(*result, 9.0)].pose.position.y > 0.0);
  REQUIRE(result->poses[nearest_x_index(*result, 9.0)].pose.position.y < cfg.eb_max_deviation);
  REQUIRE(result->poses[nearest_x_index(*result, 11.0)].pose.position.y > 0.0);
  REQUIRE(result->poses[nearest_x_index(*result, 11.0)].pose.position.y < cfg.eb_max_deviation);
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_trajectory atomically falls back to the resampled centerline and braking",
  "[trajectory_core][elastic_band]")
{
  auto cfg = make_elastic_config();
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.eb_clearance_margin = 0.0;
  trajectory_planner::TrajectoryCore core(cfg);
  const auto grid = make_tall_grid_with_full_wall_at_10m();

  const auto traj = core.compute_trajectory(make_straight_path({0.0, 10.0, 20.0}), grid, cfg.max_speed, cfg.max_speed);

  REQUIRE(traj.points.size() == 81);
  for (const auto & point : traj.points) {
    REQUIRE(point.pose.position.y == Catch::Approx(0.0).margin(1e-9));
  }
  REQUIRE(traj.points.back().max_speed == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band rejects nearby obstacles that require opposite avoidance sides",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 8.0, 0.5);
  mark_lethal_cell(grid, 10.0, -0.5);

  REQUIRE_FALSE(core.elastic_band(make_straight_path({0.0, 20.0}), grid).has_value());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band groups nearby obstacles into one smooth one-sided maneuver",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 8.0, 0.5);
  mark_lethal_cell(grid, 10.0, 0.5);

  const auto result = core.elastic_band(make_straight_path({0.0, 20.0}), grid);

  REQUIRE(result.has_value());
  double maximum_deviation = 0.0;
  for (size_t i = 0; i < result->poses.size(); ++i) {
    const double offset = result->poses[i].pose.position.y;
    REQUIRE(offset <= 1e-9);
    maximum_deviation = std::max(maximum_deviation, std::fabs(offset));
    if (i > 0) {
      REQUIRE(std::fabs(offset - result->poses[i - 1].pose.position.y) < cfg.eb_lateral_search_step);
    }
  }
  REQUIRE(maximum_deviation > 0.0);
  REQUIRE(maximum_deviation <= cfg.eb_max_deviation + 1e-9);
  REQUIRE(result->poses.front().pose.position.y == Catch::Approx(0.0).margin(1e-9));
  REQUIRE(result->poses.back().pose.position.y == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band final validation rejects a collision introduced along a transition ramp",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 10.0, 0.5);  // causes a rightward maneuver
  mark_lethal_cell(grid, 9.0, -1.5);  // clear on centerline, struck only by the entry ramp

  REQUIRE_FALSE(core.elastic_band(make_straight_path({0.0, 20.0}), grid).has_value());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "elastic_band orientations follow the deformed path tangent",
  "[trajectory_core][elastic_band]")
{
  const auto cfg = make_elastic_config();
  trajectory_planner::TrajectoryCore core(cfg);
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 10.0, 0.5);
  const auto result = core.elastic_band(make_straight_path({0.0, 20.0}), grid);
  REQUIRE(result.has_value());

  const size_t i = nearest_x_index(*result, 9.0);
  REQUIRE(i > 0);
  REQUIRE(i + 1 < result->poses.size());
  const auto & prev = result->poses[i - 1].pose.position;
  const auto & next = result->poses[i + 1].pose.position;
  const double expected_yaw = std::atan2(next.y - prev.y, next.x - prev.x);
  const auto & q = result->poses[i].pose.orientation;
  const double actual_yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
  REQUIRE(actual_yaw == Catch::Approx(expected_yaw).margin(1e-9));
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

// ---------------------------------------------------------------------------
// Path smoothing and windowed curvature
// ---------------------------------------------------------------------------

// Lanelet-style centreline: 1 m spacing, straight for 20 m, then a single heading kink.
static nav_msgs::msg::Path make_kinked_path(double kink_deg)
{
  nav_msgs::msg::Path path;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    path.poses.push_back(ps);
    if (i == 20) {
      heading = kink_deg * M_PI / 180.0;
    }
    x += std::cos(heading);
    y += std::sin(heading);
  }
  return path;
}

// Constant-radius left turn starting at the origin heading +x, 1 m chord spacing.
static nav_msgs::msg::Path make_arc_path(double radius, int count)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < count; ++i) {
    const double angle = static_cast<double>(i) / radius;
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = radius * std::sin(angle);
    ps.pose.position.y = radius - radius * std::cos(angle);
    path.poses.push_back(ps);
  }
  return path;
}

// Grid far away from every test path: everything reads as free space.
static nav_msgs::msg::OccupancyGrid make_distant_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.5;
  grid.info.width = 4;
  grid.info.height = 4;
  grid.info.origin.position.x = -100.0;
  grid.info.origin.position.y = -100.0;
  grid.data.assign(16, 0);
  return grid;
}

static trajectory_planner::TrajectoryConfig make_smoothing_config()
{
  trajectory_planner::TrajectoryConfig cfg;
  cfg.interpolation_resolution = 0.05;
  cfg.max_tangential_accel = 0.5;
  cfg.max_lateral_accel = 2.0;
  cfg.footprint_x_min = 0.0;
  cfg.footprint_x_max = 0.0;
  cfg.footprint_y_min = 0.0;
  cfg.footprint_y_max = 0.0;
  cfg.path_smoothing_enabled = true;
  cfg.curvature_window = 1.0;
  return cfg;
}

static double max_of(const std::vector<double> & values)
{
  return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "compute_curvatures recovers the radius of a densely sampled circle",
  "[trajectory_core][curvature]")
{
  auto cfg = make_smoothing_config();
  trajectory_planner::TrajectoryCore core(cfg);

  // R = 10 m sampled every 0.1 m of arc.
  nav_msgs::msg::Path path;
  for (int i = 0; i < 300; ++i) {
    const double angle = i * 0.01;
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = 10.0 * std::sin(angle);
    ps.pose.position.y = 10.0 - 10.0 * std::cos(angle);
    path.poses.push_back(ps);
  }

  const auto kappa = core.compute_curvatures(path);
  REQUIRE(kappa.size() == path.poses.size());
  for (size_t i = 0; i < kappa.size(); ++i) {
    REQUIRE(kappa[i] == Catch::Approx(0.1).margin(1e-3));
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "smooth_path leaves a straight path straight and keeps its endpoints",
  "[trajectory_core][smoothing]")
{
  auto cfg = make_smoothing_config();
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_straight_path({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0});
  auto smoothed = core.smooth_path(path);

  REQUIRE(smoothed.poses.size() > path.poses.size());
  for (const auto & ps : smoothed.poses) {
    REQUIRE(ps.pose.position.y == Catch::Approx(0.0).margin(1e-9));
  }
  REQUIRE(smoothed.poses.front().pose.position.x == Catch::Approx(0.0));
  REQUIRE(smoothed.poses.back().pose.position.x == Catch::Approx(10.0));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "smooth_path softens a centreline kink within the deviation bound",
  "[trajectory_core][smoothing]")
{
  auto cfg = make_smoothing_config();
  trajectory_planner::TrajectoryCore core(cfg);

  auto path = make_kinked_path(6.0);
  const double raw_kappa = max_of(core.compute_curvatures(path));
  auto smoothed = core.smooth_path(path);
  const double smoothed_kappa = max_of(core.compute_curvatures(smoothed));

  // A 6 degree kink reads as ~0.1 1/m over a 1 m window; smoothing should cut that by >4x.
  REQUIRE(raw_kappa > 0.09);
  REQUIRE(smoothed_kappa < raw_kappa / 4.0);

  // Every smoothed point stays within the deviation bound of the original polyline.
  const double heading = 6.0 * M_PI / 180.0;
  for (const auto & ps : smoothed.poses) {
    const double x = ps.pose.position.x;
    const double y = ps.pose.position.y;
    const double to_first_leg = std::fabs(y);
    const double to_second_leg = std::fabs(-(x - 20.0) * std::sin(heading) + y * std::cos(heading));
    const double distance = x <= 20.0 ? std::min(to_first_leg, to_second_leg) : to_second_leg;
    REQUIRE(distance <= cfg.smoothing_max_deviation + 1e-6);
  }

  REQUIRE(smoothed.poses.front().pose.position.x == Catch::Approx(path.poses.front().pose.position.x));
  REQUIRE(smoothed.poses.back().pose.position.x == Catch::Approx(path.poses.back().pose.position.x));
  REQUIRE(smoothed.poses.back().pose.position.y == Catch::Approx(path.poses.back().pose.position.y));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "smooth_path does not cut a constant-radius corner",
  "[trajectory_core][smoothing]")
{
  auto cfg = make_smoothing_config();
  trajectory_planner::TrajectoryCore core(cfg);

  const double radius = 10.0;
  auto smoothed = core.smooth_path(make_arc_path(radius, 40));
  for (const auto & ps : smoothed.poses) {
    const double r = std::hypot(ps.pose.position.x, ps.pose.position.y - radius);
    // Chords of a 1 m-sampled arc sit at most ~1.25 cm inside it; allow a little slack.
    REQUIRE(r == Catch::Approx(radius).margin(0.02));
  }
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "smoothing and windowed curvature stop a small map kink from collapsing the speed profile",
  "[trajectory_core][smoothing][speed]")
{
  // Baseline: resampling only (elastic band enabled, no smoothing) with per-segment yaw
  // curvature. At 5 cm spacing a 2 degree kink is concentrated into ~0.1 m of arc.
  auto legacy_cfg = make_smoothing_config();
  legacy_cfg.path_smoothing_enabled = false;
  legacy_cfg.curvature_window = 0.0;
  legacy_cfg.elastic_band_enabled = true;
  trajectory_planner::TrajectoryCore legacy(legacy_cfg);

  auto smooth_cfg = make_smoothing_config();
  smooth_cfg.elastic_band_enabled = true;
  trajectory_planner::TrajectoryCore smooth(smooth_cfg);

  auto path = make_kinked_path(2.0);
  auto grid = make_distant_grid();
  const double limit = 15.0;

  auto min_speed = [](const wato_trajectory_msgs::msg::Trajectory & traj) {
    double v = std::numeric_limits<double>::max();
    for (const auto & pt : traj.points) {
      v = std::min(v, pt.max_speed);
    }
    return v;
  };

  const double legacy_min = min_speed(legacy.compute_trajectory(path, grid, limit, limit));
  const double smooth_min = min_speed(smooth.compute_trajectory(path, grid, limit, limit));

  REQUIRE(legacy_min < 5.0);
  REQUIRE(smooth_min == Catch::Approx(limit).margin(1e-6));
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture,
  "obstacle deformation still applies on top of a smoothed path",
  "[trajectory_core][smoothing][elastic_band]")
{
  auto cfg = make_elastic_config();
  cfg.path_smoothing_enabled = true;
  cfg.curvature_window = 1.0;
  trajectory_planner::TrajectoryCore core(cfg);

  // Same scenario as the sparse-waypoint deformation test, routed through the full pipeline.
  auto grid = make_tall_empty_grid();
  mark_lethal_cell(grid, 10.0, 0.5);  // overlaps the left edge of the footprint
  auto traj = core.compute_trajectory(make_straight_path({0.0, 20.0}), grid, cfg.max_speed, cfg.max_speed);

  REQUIRE_FALSE(traj.points.empty());
  size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const double error = std::fabs(traj.points[i].pose.position.x - 10.0);
    if (error < best) {
      best = error;
      nearest = i;
    }
  }
  REQUIRE(traj.points[nearest].pose.position.y < -0.1);
  for (const auto & pt : traj.points) {
    REQUIRE(std::fabs(pt.pose.position.y) <= cfg.eb_max_deviation + 1e-6);
  }
}

}  // namespace wato
