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
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "fake_planner/fake_planner_core.hpp"

namespace wato
{

using fake_planner::FakePlannerConfig;
using fake_planner::FakePlannerCore;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// A maneuver body wrapped in the top-level fields, with the launch ramp and stop pad off by
// default so geometry tests see exactly the authored path. Tests that care about the speed
// profile pass their own ramp_up_accel / stop_pad_m.
static std::string maneuver(const std::string & fields, const std::string & segments)
{
  return "{" + fields + ", \"segments\": [" + segments + "]}";
}

static std::string plainFields(double spacing = 1.0, double speed = 3.0)
{
  return "\"sample_spacing_m\": " + std::to_string(spacing) + ", \"default_speed\": " + std::to_string(speed) +
         ", \"ramp_up_accel\": 0, \"stop_pad_m\": 0";
}

// Loads and anchors at the origin, which leaves window() holding the whole path -- the only way
// to inspect every waypoint through the public interface.
static FakePlannerCore built(const std::string & json, const FakePlannerConfig & config = FakePlannerConfig{})
{
  FakePlannerCore core(config);
  std::string error;
  REQUIRE(core.loadManeuverJson(json, error));
  REQUIRE(error.empty());
  core.anchor(0.0, 0.0, 0.0);
  return core;
}

static std::string loadError(const std::string & json)
{
  FakePlannerCore core(FakePlannerConfig{});
  std::string error;
  const bool ok = core.loadManeuverJson(json, error);
  REQUIRE_FALSE(ok);
  REQUIRE_FALSE(error.empty());
  return error;
}

static double yawOf(const wato_trajectory_msgs::msg::TrajectoryPoint & point)
{
  return 2.0 * std::atan2(point.pose.orientation.z, point.pose.orientation.w);
}

static double pointSpacing(
  const wato_trajectory_msgs::msg::TrajectoryPoint & a, const wato_trajectory_msgs::msg::TrajectoryPoint & b)
{
  return std::hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
}

// ---------------------------------------------------------------------------
// Schema validation
// ---------------------------------------------------------------------------

TEST_CASE("Malformed documents are rejected with a reason", "[fake_planner_core][schema]")
{
  CHECK_THAT(loadError("not json at all"), Catch::Matchers::ContainsSubstring("invalid JSON"));
  CHECK_THAT(loadError("[1, 2, 3]"), Catch::Matchers::ContainsSubstring("top-level JSON must be an object"));
  CHECK_THAT(loadError("{}"), Catch::Matchers::ContainsSubstring("segments"));
  CHECK_THAT(loadError(R"({"segments": []})"), Catch::Matchers::ContainsSubstring("segments"));
  CHECK_THAT(loadError(R"({"segments": {}})"), Catch::Matchers::ContainsSubstring("segments"));
}

TEST_CASE("Top-level numeric fields are validated", "[fake_planner_core][schema]")
{
  const std::string seg = R"({"straight": {"length": 10.0}})";

  CHECK_THAT(
    loadError(maneuver("\"sample_spacing_m\": 0", seg)),
    Catch::Matchers::ContainsSubstring("sample_spacing_m must be a finite value > 0"));
  CHECK_THAT(
    loadError(maneuver("\"sample_spacing_m\": -1.0", seg)),
    Catch::Matchers::ContainsSubstring("sample_spacing_m must be a finite value > 0"));
  CHECK_THAT(loadError(maneuver("\"default_speed\": -1.0", seg)), Catch::Matchers::ContainsSubstring("default_speed"));
  CHECK_THAT(loadError(maneuver("\"ramp_up_accel\": -1.0", seg)), Catch::Matchers::ContainsSubstring("ramp_up_accel"));
  CHECK_THAT(loadError(maneuver("\"brake_decel\": 0", seg)), Catch::Matchers::ContainsSubstring("brake_decel"));
  CHECK_THAT(loadError(maneuver("\"stop_pad_m\": -1.0", seg)), Catch::Matchers::ContainsSubstring("stop_pad_m"));
  CHECK_THAT(
    loadError(maneuver("\"start\": 5", seg)),
    Catch::Matchers::ContainsSubstring("'start' must be an object with x/y/yaw"));
}

TEST_CASE("Segment shape and type are validated", "[fake_planner_core][schema]")
{
  CHECK_THAT(
    loadError(maneuver(plainFields(), R"({"straight": {"length": 1}, "arc": {"radius": 1}})")),
    Catch::Matchers::ContainsSubstring("single {type: {...}} object"));
  CHECK_THAT(
    loadError(maneuver(plainFields(), R"({"straight": 5})")),
    Catch::Matchers::ContainsSubstring("parameters must be an object"));
  CHECK_THAT(
    loadError(maneuver(plainFields(), R"({"corkscrew": {"length": 1}})")),
    Catch::Matchers::ContainsSubstring("unknown type 'corkscrew'"));
  CHECK_THAT(
    loadError(maneuver(plainFields(), R"({"arc": {"radius": 5, "angle": 90, "dir": "sideways"}})")),
    Catch::Matchers::ContainsSubstring("dir must be 'left' or 'right'"));
  // A missing required key surfaces through nlohmann's own exception, named per segment.
  CHECK_THAT(
    loadError(maneuver(plainFields(), R"({"straight": {}})")), Catch::Matchers::ContainsSubstring("segment 1"));
}

// The reason these matter: every primitive divides by a length or a radius, so a zero there
// produces NaN or infinity that propagates through resampling and every std::min in the speed
// passes, and lands in a published max_speed. Rejecting at load is the only place it can be named.
TEST_CASE("Degenerate segment parameters are rejected rather than producing NaN", "[fake_planner_core][schema]")
{
  const std::string fields = plainFields();

  SECTION("zero and negative lengths")
  {
    CHECK_THAT(
      loadError(maneuver(fields, R"({"straight": {"length": 0}})")),
      Catch::Matchers::ContainsSubstring("'length' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"straight": {"length": -5.0}})")),
      Catch::Matchers::ContainsSubstring("'length' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"dwell": {"length": 0}})")),
      Catch::Matchers::ContainsSubstring("'length' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"shift": {"lateral": 1.0, "length": 0}})")),
      Catch::Matchers::ContainsSubstring("'length' must be a finite value > 0"));
  }

  SECTION("degenerate arcs")
  {
    CHECK_THAT(
      loadError(maneuver(fields, R"({"arc": {"radius": 0, "angle": 90}})")),
      Catch::Matchers::ContainsSubstring("'radius' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"arc": {"radius": -5.0, "angle": 90}})")),
      Catch::Matchers::ContainsSubstring("'radius' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"arc": {"radius": 10.0, "angle": 0}})")),
      Catch::Matchers::ContainsSubstring("'angle' must be a finite non-zero value"));
  }

  SECTION("degenerate slaloms")
  {
    CHECK_THAT(
      loadError(maneuver(fields, R"({"slalom": {"amplitude": 1.0, "wavelength": 0, "cycles": 2}})")),
      Catch::Matchers::ContainsSubstring("'wavelength' must be a finite value > 0"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"slalom": {"amplitude": 1.0, "wavelength": 10.0, "cycles": 0}})")),
      Catch::Matchers::ContainsSubstring("'cycles' must be a finite value > 0"));
  }

  SECTION("negative speeds")
  {
    CHECK_THAT(
      loadError(maneuver(fields, R"({"straight": {"length": 10.0, "speed": -1.0}})")),
      Catch::Matchers::ContainsSubstring("'speed' and 'end_speed'"));
    CHECK_THAT(
      loadError(maneuver(fields, R"({"straight": {"length": 10.0, "end_speed": -1.0}})")),
      Catch::Matchers::ContainsSubstring("'speed' and 'end_speed'"));
  }
}

TEST_CASE("Accepted maneuvers never contain a non-finite pose or speed", "[fake_planner_core][schema]")
{
  const std::vector<std::string> bodies = {
    R"({"straight": {"length": 20.0}})",
    R"({"dwell": {"length": 5.0}})",
    R"({"arc": {"radius": 18.0, "angle": 100.0, "dir": "left", "end_speed": 1.5}})",
    R"({"arc": {"radius": 40.25, "angle": 180.0, "dir": "right"}})",
    R"({"shift": {"lateral": -3.0, "length": 8.0}})",
    R"({"slalom": {"amplitude": 1.0, "wavelength": 12.0, "cycles": 5}})",
  };

  for (const auto & body : bodies) {
    // Once with the profile passes off, once with both on, so the ramp and pad arithmetic is
    // covered for every primitive too.
    for (const auto & fields :
         {plainFields(), std::string("\"sample_spacing_m\": 1.0, \"default_speed\": 3.0, \"ramp_up_accel\": 1.0")})
    {
      const FakePlannerCore core = built(maneuver(fields, body));
      REQUIRE(core.waypointCount() >= 2);
      for (const auto & point : core.window().points) {
        CHECK(std::isfinite(point.pose.position.x));
        CHECK(std::isfinite(point.pose.position.y));
        CHECK(std::isfinite(point.max_speed));
        CHECK(point.max_speed >= 0.0F);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Resampling
// ---------------------------------------------------------------------------

TEST_CASE("Waypoints are laid out at sample_spacing_m and finish on the authored end", "[fake_planner_core][path]")
{
  const double spacing = GENERATE(0.5, 1.0, 2.0);
  const FakePlannerCore core = built(maneuver(plainFields(spacing), R"({"straight": {"length": 20.0}})"));
  const auto & points = core.window().points;

  REQUIRE(points.size() >= 2);
  // Interior gaps are exactly the requested spacing; only the final stitch to the authored
  // endpoint may be short.
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    CHECK(pointSpacing(points[i - 1], points[i]) == Catch::Approx(spacing).margin(1e-6));
  }
  CHECK(pointSpacing(points[points.size() - 2], points.back()) <= spacing + 1e-6);

  CHECK(points.front().pose.position.x == Catch::Approx(0.0).margin(1e-9));
  CHECK(points.back().pose.position.x == Catch::Approx(20.0).margin(1e-6));
  CHECK(core.pathLength() == Catch::Approx(20.0).margin(1e-6));
}

// ---------------------------------------------------------------------------
// Segment geometry
// ---------------------------------------------------------------------------

TEST_CASE("straight advances along the heading", "[fake_planner_core][path]")
{
  const FakePlannerCore core = built(maneuver(plainFields(), R"({"straight": {"length": 12.0}})"));
  for (const auto & point : core.window().points) {
    CHECK(point.pose.position.y == Catch::Approx(0.0).margin(1e-9));
    CHECK(yawOf(point) == Catch::Approx(0.0).margin(1e-9));
  }
}

TEST_CASE("arc holds its radius and rotates the heading", "[fake_planner_core][path]")
{
  const double radius = 10.0;

  SECTION("left turns curve toward +y")
  {
    const FakePlannerCore core =
      built(maneuver(plainFields(0.5), R"({"arc": {"radius": 10.0, "angle": 90.0, "dir": "left"}})"));
    const auto & points = core.window().points;
    // A left arc from the origin heading +x sweeps about (0, radius).
    for (const auto & point : points) {
      const double r = std::hypot(point.pose.position.x, point.pose.position.y - radius);
      CHECK(r == Catch::Approx(radius).margin(1e-3));
    }
    CHECK(points.back().pose.position.x == Catch::Approx(radius).margin(1e-2));
    CHECK(points.back().pose.position.y == Catch::Approx(radius).margin(1e-2));
    CHECK(core.pathLength() == Catch::Approx(radius * M_PI / 2.0).margin(1e-2));
  }

  SECTION("right turns mirror left ones")
  {
    const FakePlannerCore core =
      built(maneuver(plainFields(0.5), R"({"arc": {"radius": 10.0, "angle": 90.0, "dir": "right"}})"));
    const auto & points = core.window().points;
    for (const auto & point : points) {
      const double r = std::hypot(point.pose.position.x, point.pose.position.y + radius);
      CHECK(r == Catch::Approx(radius).margin(1e-3));
    }
    CHECK(points.back().pose.position.y == Catch::Approx(-radius).margin(1e-2));
  }

  SECTION("the heading carries into the next segment")
  {
    // A quarter turn left then a straight: the straight must run along +y, proving the cursor's
    // heading was rotated and not just its position moved.
    const FakePlannerCore core = built(maneuver(
      plainFields(0.5), R"({"arc": {"radius": 10.0, "angle": 90.0, "dir": "left"}}, {"straight": {"length": 10.0}})"));
    const auto & end = core.window().points.back();
    CHECK(end.pose.position.x == Catch::Approx(10.0).margin(1e-2));
    CHECK(end.pose.position.y == Catch::Approx(20.0).margin(1e-2));
  }
}

TEST_CASE("shift reaches its offset tangent to the heading", "[fake_planner_core][path]")
{
  const FakePlannerCore core = built(maneuver(plainFields(0.5), R"({"shift": {"lateral": 2.0, "length": 10.0}})"));
  const auto & points = core.window().points;

  CHECK(points.back().pose.position.x == Catch::Approx(10.0).margin(1e-6));
  CHECK(points.back().pose.position.y == Catch::Approx(2.0).margin(1e-6));
  // Tangent at both ends: the raised cosine has zero slope there, so the first and last
  // headings stay near the original.
  CHECK(std::abs(yawOf(points[1])) < 0.1);
  CHECK(std::abs(yawOf(points[points.size() - 2])) < 0.1);
  // And it never overshoots the requested offset.
  for (const auto & point : points) {
    CHECK(point.pose.position.y <= 2.0 + 1e-6);
    CHECK(point.pose.position.y >= -1e-6);
  }
}

TEST_CASE("slalom weaves within its amplitude and returns to the centreline", "[fake_planner_core][path]")
{
  const FakePlannerCore core =
    built(maneuver(plainFields(0.5), R"({"slalom": {"amplitude": 1.0, "wavelength": 12.0, "cycles": 2}})"));
  const auto & points = core.window().points;

  CHECK(points.front().pose.position.y == Catch::Approx(0.0).margin(1e-9));
  CHECK(points.back().pose.position.y == Catch::Approx(0.0).margin(1e-6));
  double peak = 0.0;
  for (const auto & point : points) {
    peak = std::max(peak, std::abs(point.pose.position.y));
  }
  CHECK(peak <= 1.0 + 1e-6);
  // The middle of a 2-cycle weave is past the taper, so it should reach close to full amplitude.
  CHECK(peak > 0.9);
}

// ---------------------------------------------------------------------------
// Launch ramp
// ---------------------------------------------------------------------------

TEST_CASE("The launch ramp starts from rest under the accel envelope", "[fake_planner_core][speed]")
{
  const double accel = 1.0;
  const double cruise = 3.0;
  const FakePlannerCore core = built(maneuver(
    "\"sample_spacing_m\": 0.5, \"default_speed\": 3.0, \"ramp_up_accel\": 1.0, \"stop_pad_m\": 0",
    R"({"straight": {"length": 30.0}})"));
  const auto & points = core.window().points;

  CHECK(points.front().max_speed == Catch::Approx(0.0).margin(1e-9));

  double s = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s += pointSpacing(points[i - 1], points[i]);
      // Never faster than the ramp allows, and never slower than the previous point.
      CHECK(points[i].max_speed >= points[i - 1].max_speed - 1e-6);
    }
    CHECK(points[i].max_speed <= std::min(cruise, std::sqrt(2.0 * accel * s)) + 1e-6);
  }
  // v^2 / 2a = 4.5 m at these numbers, so a 30 m straight is at cruise well before the end.
  CHECK(points.back().max_speed == Catch::Approx(cruise).margin(1e-6));
}

TEST_CASE("ramp_up_accel of zero publishes the authored speed immediately", "[fake_planner_core][speed]")
{
  const FakePlannerCore core = built(maneuver(plainFields(), R"({"straight": {"length": 20.0}})"));
  CHECK(core.window().points.front().max_speed == Catch::Approx(3.0).margin(1e-6));
}

TEST_CASE("A closed maneuver gets neither ramp nor pad", "[fake_planner_core][speed]")
{
  const FakePlannerCore core = built(maneuver(
    "\"sample_spacing_m\": 2.0, \"default_speed\": 3.0, \"closed\": true",
    R"({"arc": {"radius": 20.0, "angle": 360.0, "dir": "left"}})"));

  REQUIRE(core.closed());
  for (const auto & point : core.window().points) {
    CHECK(point.max_speed == Catch::Approx(3.0).margin(1e-6));
  }
  // No end to stop at, so the node's finish check can never fire.
  CHECK(std::isinf(core.distanceToEnd()));
}

// ---------------------------------------------------------------------------
// Stop pad
// ---------------------------------------------------------------------------

TEST_CASE("The stop pad brakes into the stop line and extends past it", "[fake_planner_core][speed]")
{
  const double pad_m = 10.0;
  const double decel = 1.0;
  const FakePlannerCore core = built(maneuver(
    "\"sample_spacing_m\": 0.5, \"default_speed\": 3.0, \"ramp_up_accel\": 0, "
    "\"stop_pad_m\": 10.0, \"brake_decel\": 1.0",
    R"({"straight": {"length": 30.0}})"));
  const auto & points = core.window().points;

  // The pad is geometry past the authored end, all of it at zero speed.
  CHECK(core.pathLength() == Catch::Approx(30.0 + pad_m).margin(1.0));
  CHECK(points.back().max_speed == Catch::Approx(0.0).margin(1e-9));

  // Walking back from the end: distance from the last positive-speed point to the path end is the
  // pad plus the stop line's own point.
  double zero_run = 0.0;
  for (std::size_t i = points.size() - 1; i > 0; --i) {
    if (points[i - 1].max_speed > 0.0F) {
      break;
    }
    zero_run += pointSpacing(points[i - 1], points[i]);
  }
  CHECK(zero_run >= pad_m - 1e-6);

  // Speeds ahead of the stop line respect the braking envelope: v <= sqrt(2 d s) for the distance
  // s remaining to the first zero-speed point.
  std::size_t stop = points.size() - 1;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (points[i].max_speed == 0.0F) {
      stop = i;
      break;
    }
  }
  REQUIRE(stop > 0);

  double remaining = 0.0;
  for (std::size_t i = stop; i > 0; --i) {
    remaining += pointSpacing(points[i - 1], points[i]);
    CHECK(points[i - 1].max_speed <= std::sqrt(2.0 * decel * remaining) + 1e-6);
  }
}

TEST_CASE("distanceToEnd measures to the stop line, not the end of the pad", "[fake_planner_core][speed]")
{
  FakePlannerCore core = built(maneuver(
    "\"sample_spacing_m\": 1.0, \"default_speed\": 3.0, \"ramp_up_accel\": 0, \"stop_pad_m\": 10.0",
    R"({"straight": {"length": 30.0}})"));

  core.updateWindow(0.0, 0.0);
  CHECK(core.distanceToEnd() == Catch::Approx(30.0).margin(1.0));
  // The pad accounts for the difference between the whole path and the drivable part.
  CHECK(core.pathLength() - core.distanceToEnd() == Catch::Approx(10.0).margin(1.0));

  // Partway along, what is left shrinks by what has been driven.
  core.updateWindow(20.0, 0.0);
  CHECK(core.distanceToEnd() == Catch::Approx(10.0).margin(1.0));

  // Into the pad, "distance remaining" clamps at zero rather than going negative.
  core.updateWindow(35.0, 0.0);
  CHECK(core.distanceToEnd() == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("A larger brake_decel stops in a shorter distance", "[fake_planner_core][speed]")
{
  const auto braking_distance = [](const std::string & decel) {
    const FakePlannerCore core = built(maneuver(
      "\"sample_spacing_m\": 0.5, \"default_speed\": 5.0, \"ramp_up_accel\": 0, "
      "\"stop_pad_m\": 10.0, \"brake_decel\": " +
        decel,
      R"({"straight": {"length": 40.0}})"));
    const auto & points = core.window().points;
    // Distance from where the profile first drops below cruise to where it reaches zero.
    double distance = 0.0;
    bool braking = false;
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (!braking && points[i].max_speed < 5.0F - 1e-3F) {
        braking = true;
      }
      if (braking) {
        distance += pointSpacing(points[i - 1], points[i]);
      }
      if (points[i].max_speed == 0.0F) {
        break;
      }
    }
    return distance;
  };

  CHECK(braking_distance("1.0") > braking_distance("4.0"));
}

// ---------------------------------------------------------------------------
// Anchoring
// ---------------------------------------------------------------------------

TEST_CASE("anchor is a rigid transform", "[fake_planner_core][anchor]")
{
  const std::string json =
    maneuver(plainFields(), R"({"straight": {"length": 6.0}}, {"shift": {"lateral": 2.0, "length": 8.0}})");

  const FakePlannerCore at_origin = built(json);
  FakePlannerCore moved(FakePlannerConfig{});
  std::string error;
  REQUIRE(moved.loadManeuverJson(json, error));
  moved.anchor(10.0, -5.0, 0.7);

  const auto & a = at_origin.window().points;
  const auto & b = moved.window().points;
  REQUIRE(a.size() == b.size());

  CHECK(b.front().pose.position.x == Catch::Approx(10.0).margin(1e-9));
  CHECK(b.front().pose.position.y == Catch::Approx(-5.0).margin(1e-9));

  // Distances between points, and speeds, survive the transform unchanged.
  for (std::size_t i = 1; i < a.size(); ++i) {
    CHECK(pointSpacing(b[i - 1], b[i]) == Catch::Approx(pointSpacing(a[i - 1], a[i])).margin(1e-9));
    CHECK(b[i].max_speed == Catch::Approx(a[i].max_speed).margin(1e-9));
  }
  // Every heading is rotated by exactly the anchor yaw.
  CHECK(yawOf(b.front()) - yawOf(a.front()) == Catch::Approx(0.7).margin(1e-9));
}

TEST_CASE("An absolute start marks the maneuver as fixed geometry", "[fake_planner_core][anchor]")
{
  const FakePlannerCore relative = built(maneuver(plainFields(), R"({"straight": {"length": 10.0}})"));
  CHECK_FALSE(relative.hasAbsoluteStart());

  const FakePlannerCore absolute = built(maneuver(
    plainFields() + R"(, "start": {"x": -42.0, "y": -40.25, "yaw": 0.0})", R"({"straight": {"length": 10.0}})"));
  CHECK(absolute.hasAbsoluteStart());
  // Anchoring such a maneuver at the origin is the identity, so it lands where it was drawn.
  CHECK(absolute.window().points.front().pose.position.x == Catch::Approx(-42.0).margin(1e-9));
  CHECK(absolute.window().points.front().pose.position.y == Catch::Approx(-40.25).margin(1e-9));
}

TEST_CASE("The core is not ready until anchored, and clear undoes it", "[fake_planner_core][anchor]")
{
  FakePlannerCore core(FakePlannerConfig{});
  std::string error;

  CHECK_FALSE(core.ready());
  CHECK(std::isinf(core.distanceToEnd()));

  REQUIRE(core.loadManeuverJson(maneuver(plainFields(), R"({"straight": {"length": 20.0}})"), error));
  // Loading alone does not anchor: the waypoints exist but there is nothing to publish yet.
  CHECK_FALSE(core.ready());
  CHECK(core.waypointCount() >= 2);

  core.anchor(0.0, 0.0, 0.0);
  CHECK(core.ready());

  core.clear();
  CHECK_FALSE(core.ready());
  CHECK(core.window().points.empty());
  // The loaded waypoints survive, so a re-anchor needs no reload.
  CHECK(core.waypointCount() >= 2);
  core.anchor(1.0, 2.0, 0.0);
  CHECK(core.ready());
}

// ---------------------------------------------------------------------------
// Rolling window
// ---------------------------------------------------------------------------

TEST_CASE("The window spans trail_m behind through horizon_m ahead", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 10.0;
  config.trail_m = 2.0;
  FakePlannerCore core = built(maneuver(plainFields(0.5), R"({"straight": {"length": 60.0}})"), config);

  core.updateWindow(30.0, 0.0);
  const auto & points = core.window().points;
  REQUIRE(points.size() >= 2);

  // Some path behind the vehicle, because pure pursuit needs it to avoid creeping...
  const double behind = 30.0 - points.front().pose.position.x;
  CHECK(behind >= config.trail_m - 0.5);
  CHECK(behind <= config.trail_m + 0.5);
  // ...and at least the horizon ahead of it.
  CHECK(points.back().pose.position.x - 30.0 >= config.horizon_m - 0.5);

  // The window is a slice, not the whole route.
  CHECK(points.size() < core.waypointCount());
}

TEST_CASE("An open window clamps at the end of the path", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 20.0;
  config.trail_m = 2.0;
  FakePlannerCore core = built(
    maneuver(
      "\"sample_spacing_m\": 1.0, \"default_speed\": 3.0, \"ramp_up_accel\": 0, \"stop_pad_m\": 0",
      R"({"straight": {"length": 30.0}})"),
    config);

  core.updateWindow(29.0, 0.0);
  const auto & points = core.window().points;
  // Nothing past the authored end exists to publish, so the window simply runs out.
  CHECK(points.back().pose.position.x == Catch::Approx(30.0).margin(1e-6));
  CHECK(points.size() < 25);
}

TEST_CASE("A closed window wraps at the seam so the vehicle laps", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 20.0;
  config.trail_m = 4.0;
  const double radius = 20.0;
  FakePlannerCore core = built(
    maneuver(
      "\"sample_spacing_m\": 1.0, \"default_speed\": 3.0, \"closed\": true",
      R"({"arc": {"radius": 20.0, "angle": 360.0, "dir": "left"}})"),
    config);

  // Drive a full lap plus a bit, sampling the circle. The window must stay a full-length slice the
  // whole way round -- if it did not wrap, it would collapse as the vehicle neared the seam.
  const std::size_t first_size = [&]() {
    core.updateWindow(0.0, 0.0);
    return core.window().points.size();
  }();
  REQUIRE(first_size > 10);

  for (int step = 0; step <= 40; ++step) {
    const double theta = 2.0 * M_PI * static_cast<double>(step) / 36.0;
    // Parametrisation of the arc: centre (0, radius), starting at the origin heading +x.
    core.updateWindow(radius * std::sin(theta), radius * (1.0 - std::cos(theta)));
    const auto size = core.window().points.size();
    CHECK(size >= first_size - 2);
    CHECK(size <= first_size + 2);
  }
}

TEST_CASE("The window does not snap back on a self-crossing maneuver", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 10.0;
  config.trail_m = 2.0;
  // A slalom passes close to its own earlier points; a global nearest search would jump back.
  FakePlannerCore core = built(
    maneuver(
      plainFields(0.5),
      R"({"straight": {"length": 6.0}}, {"slalom": {"amplitude": 1.0, "wavelength": 12.0, "cycles": 5}}, )"
      R"({"straight": {"length": 6.0}})"),
    config);

  // Follow the path itself, which is the worst case for ambiguity, and check that progress toward
  // the end never reverses.
  const auto full = core.window().points;
  double previous = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < full.size(); i += 3) {
    core.updateWindow(full[i].pose.position.x, full[i].pose.position.y);
    const double remaining = core.distanceToEnd();
    CHECK(remaining <= previous + 1e-6);
    previous = remaining;
  }
}

TEST_CASE("distanceToPath reports how far the vehicle has strayed", "[fake_planner_core][window]")
{
  FakePlannerCore core = built(maneuver(plainFields(0.5), R"({"straight": {"length": 40.0}})"));

  // Before any slice there is no match to report.
  CHECK(std::isinf(core.distanceToPath()));

  core.updateWindow(20.0, 0.0);
  CHECK(core.distanceToPath() == Catch::Approx(0.0).margin(1e-6));

  core.updateWindow(20.0, 5.0);
  CHECK(core.distanceToPath() == Catch::Approx(5.0).margin(1e-6));
}

TEST_CASE("search_ahead_m bounds how far ahead a match is looked for", "[fake_planner_core][window]")
{
  const std::string json = maneuver(plainFields(1.0), R"({"straight": {"length": 300.0}})");

  SECTION("a vehicle beyond the search window is not picked up")
  {
    FakePlannerConfig config;
    config.search_ahead_m = 20.0;
    FakePlannerCore core = built(json, config);
    core.updateWindow(250.0, 0.0);
    // The match is pinned to the end of the searched stretch, not the vehicle's actual position.
    CHECK(core.distanceToPath() > 200.0);
  }

  SECTION("widening the search finds it")
  {
    FakePlannerConfig config;
    config.search_ahead_m = 400.0;
    FakePlannerCore core = built(json, config);
    core.updateWindow(250.0, 0.0);
    CHECK(core.distanceToPath() == Catch::Approx(0.0).margin(1e-6));
  }

  SECTION("the reach is in metres, so it survives a change of spacing")
  {
    // The same 100 m of lookahead over a maneuver sampled twice as coarsely.
    FakePlannerConfig config;
    config.search_ahead_m = 100.0;
    FakePlannerCore fine = built(maneuver(plainFields(0.5), R"({"straight": {"length": 300.0}})"), config);
    FakePlannerCore coarse = built(maneuver(plainFields(2.0), R"({"straight": {"length": 300.0}})"), config);
    fine.updateWindow(90.0, 0.0);
    coarse.updateWindow(90.0, 0.0);
    CHECK(fine.distanceToPath() == Catch::Approx(0.0).margin(1e-6));
    CHECK(coarse.distanceToPath() == Catch::Approx(0.0).margin(1e-6));
  }
}

TEST_CASE("rewind sends the window back to the start of the path", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 10.0;
  config.trail_m = 2.0;
  FakePlannerCore core = built(maneuver(plainFields(1.0), R"({"straight": {"length": 100.0}})"), config);

  const double full = core.distanceToEnd();
  core.updateWindow(80.0, 0.0);
  REQUIRE(core.distanceToEnd() < 30.0);

  // Rewinding puts the published window back at the head of the path.
  core.rewind();
  CHECK(core.distanceToEnd() == Catch::Approx(full).margin(1e-9));

  // But the next slice re-acquires the nearest point, and the vehicle is still parked at the far
  // end, so an untouched maneuver snaps straight back to where it finished. This is exactly why an
  // absolute (non-anchoring) open maneuver cannot be re-run in place by the reset service -- the
  // node warns about that pairing at configure time.
  core.updateWindow(80.0, 0.0);
  CHECK(core.distanceToEnd() < 30.0);
}

// A trail longer than the whole path used to walk backwards without any iteration bound: on a
// closed maneuver the walk wraps, so it kept going round accumulating distance until it had
// covered trail_m. At this magnitude that is hundreds of billions of iterations inside an odom
// callback -- a hang, from nothing worse than an over-large trail_m in params.yaml.
TEST_CASE("An absurd trail_m cannot spin the backward walk", "[fake_planner_core][window]")
{
  FakePlannerConfig config;
  config.horizon_m = 20.0;
  config.trail_m = 1.0e12;
  FakePlannerCore core = built(
    maneuver(
      "\"sample_spacing_m\": 1.0, \"default_speed\": 3.0, \"closed\": true",
      R"({"arc": {"radius": 20.0, "angle": 360.0, "dir": "left"}})"),
    config);

  core.updateWindow(0.0, 0.0);
  // Bounded by the path itself: at most one pass over every point.
  CHECK(core.window().points.size() <= core.waypointCount());
}

}  // namespace wato
