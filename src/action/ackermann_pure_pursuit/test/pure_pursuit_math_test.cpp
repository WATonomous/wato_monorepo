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

#include "ackermann_pure_pursuit/pure_pursuit_math.hpp"

#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using ackermann_pure_pursuit::math::Vec2;
using Catch::Approx;
namespace ppm = ackermann_pure_pursuit::math;

TEST_CASE("cross-track error is signed with left positive", "[tracking]")
{
  // Path is the line y = -1 (below/behind-left of a vehicle travelling +x):
  // the vehicle (origin) is to the LEFT of the path direction -> positive CTE.
  std::vector<Vec2> below{{0.0, -1.0}, {5.0, -1.0}};
  auto e = ppm::computeTrackingError(below);
  REQUIRE(e.valid);
  CHECK(e.cross_track == Approx(1.0));
  CHECK(e.heading == Approx(0.0));

  // Path above the vehicle -> vehicle is to the RIGHT -> negative CTE.
  std::vector<Vec2> above{{0.0, 1.0}, {5.0, 1.0}};
  auto e2 = ppm::computeTrackingError(above);
  REQUIRE(e2.valid);
  CHECK(e2.cross_track == Approx(-1.0));
}

TEST_CASE("heading error reflects path tangent in vehicle frame", "[tracking]")
{
  // Path heading 45 deg to the left.
  std::vector<Vec2> diag{{1.0, 1.0}, {2.0, 2.0}};
  auto e = ppm::computeTrackingError(diag);
  REQUIRE(e.valid);
  CHECK(e.heading == Approx(M_PI / 4.0));
}

TEST_CASE("curvature from three points matches 1/R with sign", "[curvature]")
{
  // Three points on a circle of radius 5 centred at (0, 5); the arc bends left.
  Vec2 a{-3.0, 1.0};
  Vec2 b{0.0, 0.0};
  Vec2 c{3.0, 1.0};
  double k = ppm::curvatureFromPoints(a, b, c);
  CHECK(k == Approx(0.2));  // +1/5, left turn positive

  // Reverse the winding -> opposite sign.
  double k_rev = ppm::curvatureFromPoints(c, b, a);
  CHECK(k_rev == Approx(-0.2));

  // Collinear points -> zero curvature.
  CHECK(ppm::curvatureFromPoints({0, 0}, {1, 0}, {2, 0}) == Approx(0.0));
}

TEST_CASE("lookahead point is interpolated to the exact radius", "[lookahead]")
{
  std::vector<Vec2> straight{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  auto r = ppm::findLookaheadPoint(straight, 2.5, 0.5);
  REQUIRE(r.found);
  CHECK(r.point.x == Approx(2.5));
  CHECK(r.point.y == Approx(0.0));
  CHECK(r.distance == Approx(2.5));
}

TEST_CASE("lookahead falls back to last forward point when path is too short", "[lookahead]")
{
  std::vector<Vec2> shortpath{{0, 0}, {1, 0}, {2, 0}};
  auto r = ppm::findLookaheadPoint(shortpath, 10.0, 0.5);
  REQUIRE(r.found);
  CHECK(r.point.x == Approx(2.0));
}

TEST_CASE("lookahead ignores points behind the vehicle", "[lookahead]")
{
  std::vector<Vec2> path{{-3, 0}, {-1, 0}, {1, 0}, {3, 0}};
  auto r = ppm::findLookaheadPoint(path, 2.0, 0.5);
  REQUIRE(r.found);
  CHECK(r.point.x == Approx(2.0));
  CHECK(r.point.x > 0.0);
}

TEST_CASE("lookahead interpolates across the behind-to-forward boundary", "[lookahead]")
{
  // The segment (-3,0) -> (3,0) crosses the lookahead circle at (2.5, 0); the
  // target must not snap to the vertex (3, 0).
  std::vector<Vec2> path{{-3.0, 0.0}, {3.0, 0.0}};
  auto r = ppm::findLookaheadPoint(path, 2.5, 0.5);
  REQUIRE(r.found);
  CHECK(r.point.x == Approx(2.5));
  CHECK(r.point.y == Approx(0.0));
  CHECK(r.distance == Approx(2.5));
}

TEST_CASE("lookahead start index skips an earlier path lobe", "[lookahead]")
{
  // A stale/overlapping first point sits beyond ld; anchoring the search past it
  // must pick the crossing on the later portion of the path instead.
  std::vector<Vec2> path{{3.0, 3.0}, {0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  auto stale = ppm::findLookaheadPoint(path, 2.5, 0.5);
  REQUIRE(stale.found);
  CHECK(stale.point.y == Approx(3.0));  // without anchoring, matches the stale lobe

  auto anchored = ppm::findLookaheadPoint(path, 2.5, 0.5, 1);
  REQUIRE(anchored.found);
  CHECK(anchored.point.x == Approx(2.5));
  CHECK(anchored.point.y == Approx(0.0));
}

TEST_CASE("min speed within horizon does not skip a stop inside it", "[speed]")
{
  std::vector<Vec2> path{{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0}};
  std::vector<double> speeds{5.0, 0.0, 5.0, 5.0, 5.0};
  // A commanded stop at x=2 lies inside the 4 m horizon: min must be 0.
  CHECK(ppm::minSpeedWithinHorizon(path, speeds, 4.0, 99.0) == Approx(0.0));

  // All-fast path: min equals the commanded speed, not the fallback.
  std::vector<double> fast{3.0, 3.0, 3.0, 3.0, 3.0};
  CHECK(ppm::minSpeedWithinHorizon(path, fast, 4.0, 99.0) == Approx(3.0));

  // Sampling stops at the first point past the horizon: a slow point beyond it
  // does not drag the result down.
  std::vector<double> slow_later{3.0, 3.0, 3.0, 3.0, 0.5};
  CHECK(ppm::minSpeedWithinHorizon(path, slow_later, 2.5, 99.0) == Approx(3.0));
}

TEST_CASE("min speed within horizon respects an end-of-path stop", "[speed]")
{
  // Path ends inside the horizon; the tapering-to-zero speeds must bound the
  // result instead of falling back to max speed.
  std::vector<Vec2> path{{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  std::vector<double> speeds{2.0, 1.0, 0.0};
  CHECK(ppm::minSpeedWithinHorizon(path, speeds, 10.0, 99.0) == Approx(0.0));

  // No forward points at all -> fallback.
  std::vector<Vec2> behind{{-2.0, 0.0}, {-1.0, 0.0}};
  std::vector<double> bspeeds{1.0, 1.0};
  CHECK(ppm::minSpeedWithinHorizon(behind, bspeeds, 10.0, 99.0) == Approx(99.0));
}

TEST_CASE("arc-length curvature is insensitive to waypoint density", "[curvature]")
{
  // Sample a radius-5 left-turning circle at two densities; both estimates
  // should recover kappa = +0.2.
  auto make_circle = [](double step_rad, int n) {
    std::vector<Vec2> path;
    for (int i = 0; i < n; ++i) {
      double a = i * step_rad;
      path.push_back({5.0 * std::sin(a), 5.0 * (1.0 - std::cos(a))});
    }
    return path;
  };
  auto dense = make_circle(0.02, 100);  // ~0.1 m spacing
  auto sparse = make_circle(0.2, 10);  // ~1.0 m spacing

  double k_dense = ppm::curvatureByArc(dense, 50, 1.5);
  double k_sparse = ppm::curvatureByArc(sparse, 5, 1.5);
  CHECK(k_dense == Approx(0.2).margin(0.01));
  CHECK(k_sparse == Approx(0.2).margin(0.01));

  // Straight path -> zero curvature; window clamps at the path ends.
  std::vector<Vec2> straight{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  CHECK(ppm::curvatureByArc(straight, 0, 1.5) == Approx(0.0));
  CHECK(ppm::curvatureByArc(straight, 3, 1.5) == Approx(0.0));
}

TEST_CASE("curvature-limited speed respects the lateral-accel budget", "[speed]")
{
  // v = sqrt(a_lat / |kappa|) = sqrt(2.0 / 0.5) = 2.0, below the cap.
  CHECK(ppm::curvatureLimitedSpeed(2.0, 0.5, 5.0) == Approx(2.0));
  // Straight path -> no limit, return the cap.
  CHECK(ppm::curvatureLimitedSpeed(2.0, 0.0, 5.0) == Approx(5.0));
  // Gentle curve whose limit exceeds the cap -> capped.
  CHECK(ppm::curvatureLimitedSpeed(2.0, 0.01, 5.0) == Approx(5.0));
}

TEST_CASE("slew limit bounds the per-step change", "[shaping]")
{
  CHECK(ppm::slewLimit(0.0, 5.0, 1.0) == Approx(1.0));
  CHECK(ppm::slewLimit(5.0, 0.0, 1.0) == Approx(4.0));
  CHECK(ppm::slewLimit(2.0, 2.3, 1.0) == Approx(2.3));  // within step -> unchanged
}

TEST_CASE("cross-track steer correction pushes back toward the path", "[shaping]")
{
  // Vehicle left of path (cte > 0) -> steer right (negative correction).
  CHECK(ppm::crossTrackSteerCorrection(1.0, 0.0, 1.0, 1.0) == Approx(-M_PI / 4.0));
  // Vehicle right of path (cte < 0) -> steer left (positive correction).
  CHECK(ppm::crossTrackSteerCorrection(-1.0, 0.0, 1.0, 1.0) == Approx(M_PI / 4.0));
  // Correction shrinks as speed rises (softening).
  double slow = std::abs(ppm::crossTrackSteerCorrection(1.0, 0.0, 1.0, 1.0));
  double fast = std::abs(ppm::crossTrackSteerCorrection(1.0, 10.0, 1.0, 1.0));
  CHECK(fast < slow);
}

TEST_CASE("pursuit curvature matches the geometric formula", "[lookahead]")
{
  CHECK(ppm::pursuitCurvature({3.0, 0.0}) == Approx(0.0));
  CHECK(ppm::pursuitCurvature({2.0, 2.0}) == Approx(0.5));  // 2*y/(x^2+y^2) = 4/8
}

TEST_CASE("horizon curvature scan catches a sharp section a single probe skips", "[speed_budget]")
{
  // Path: straight to x=3, then a sharp U-shaped arc of R = 0.5 (|kappa| = 2),
  // then continues straight at y=1. Probe range [2, 5]:
  //   - N=1 probes only at ld=5, which lands on the straight AFTER the arc -> kappa=0.
  //   - N=3 probes at {2, 3.5, 5}: the 3.5 probe lands on the arc -> kappa >> 0.
  // This is the exact "single lookahead straddles the curve" case the change is
  // supposed to fix.
  std::vector<Vec2> path;
  for (double x = 0.0; x <= 3.0; x += 0.5) {
    path.push_back({x, 0.0});
  }
  const double cx = 3.0, cy = 0.5, R = 0.5;
  for (double theta = -M_PI / 2.0; theta <= M_PI / 2.0; theta += 0.1) {
    path.push_back({cx + R * std::cos(theta), cy + R * std::sin(theta)});
  }
  for (double x = 3.0; x <= 8.0; x += 0.5) {
    path.push_back({x, 1.0});
  }

  const double min_ld = 2.0;
  const double max_ld = 5.0;
  const double arc = 1.0;

  double kappa_single = ppm::maxAbsCurvatureOverHorizon(path, min_ld, max_ld, 1, arc);
  double kappa_multi = ppm::maxAbsCurvatureOverHorizon(path, min_ld, max_ld, 3, arc);

  CHECK(kappa_single == Approx(0.0));  // single probe misses the arc
  CHECK(kappa_multi > 1.0);  // multi-probe catches it (true |kappa| ~ 2/R = 2)
}

TEST_CASE("horizon curvature scan is a no-op on straight paths", "[speed_budget]")
{
  std::vector<Vec2> straight;
  for (double x = 0.0; x <= 10.0; x += 0.5) {
    straight.push_back({x, 0.0});
  }
  double k = ppm::maxAbsCurvatureOverHorizon(straight, 2.0, 5.0, 5, 1.0);
  CHECK(k == Approx(0.0));
}

TEST_CASE("horizon curvature scan with probe_count=1 uses the max distance only", "[speed_budget]")
{
  // Straight then bend at ~5 m: a single probe at max_ld should sample the bend,
  // while a probe placed only at min_ld would miss it. Confirms our N=1 semantics.
  std::vector<Vec2> path;
  for (double x = 0.0; x <= 4.5; x += 0.5) {
    path.push_back({x, 0.0});
  }
  const double cx = 4.5, cy = 2.0, R = 2.0;
  for (double theta = -M_PI / 2.0; theta <= 0.0; theta += 0.1) {
    path.push_back({cx + R * std::cos(theta), cy + R * std::sin(theta)});
  }
  double k = ppm::maxAbsCurvatureOverHorizon(path, 2.0, 5.0, 1, 1.0);
  CHECK(k > 0.0);  // saw the bend
}

TEST_CASE("horizon curvature scan tolerates degenerate inputs", "[speed_budget]")
{
  // Empty path -> 0
  CHECK(ppm::maxAbsCurvatureOverHorizon({}, 2.0, 5.0, 3, 1.0) == Approx(0.0));
  // Zero probes -> 0
  std::vector<Vec2> path{{0, 0}, {1, 0}, {2, 0}};
  CHECK(ppm::maxAbsCurvatureOverHorizon(path, 2.0, 5.0, 0, 1.0) == Approx(0.0));
}
