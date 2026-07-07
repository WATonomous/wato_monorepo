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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "ackermann_pure_pursuit/pure_pursuit_math.hpp"

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
