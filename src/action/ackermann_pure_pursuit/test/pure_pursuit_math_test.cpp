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

#include <algorithm>
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

// --------------------------------------------------------------------------
// Multi-look-ahead (MLPP)
// --------------------------------------------------------------------------

namespace
{

// Straight path along +x, 0.5 m spacing.
std::vector<Vec2> straightPath(double length = 12.0)
{
  std::vector<Vec2> path;
  for (double x = 0.0; x <= length; x += 0.5) {
    path.push_back({x, 0.0});
  }
  return path;
}

// Constant-radius arc starting at the origin and turning left with radius R.
std::vector<Vec2> arcPath(double radius, double sweep = 1.4, double step = 0.02)
{
  std::vector<Vec2> path;
  for (double theta = 0.0; theta < sweep; theta += step) {
    path.push_back({radius * std::sin(theta), radius * (1.0 - std::cos(theta))});
  }
  return path;
}

}  // namespace

TEST_CASE("linear spacing spans the range inclusively", "[mlpp]")
{
  auto three = ppm::linearSpacing(2.0, 5.0, 3);
  REQUIRE(three.size() == 3);
  CHECK(three[0] == Approx(2.0));
  CHECK(three[1] == Approx(3.5));
  CHECK(three[2] == Approx(5.0));

  // A single sample keeps the far-end semantics of the single-lookahead law.
  auto one = ppm::linearSpacing(2.0, 5.0, 1);
  REQUIRE(one.size() == 1);
  CHECK(one[0] == Approx(5.0));

  CHECK(ppm::linearSpacing(2.0, 5.0, 0).empty());

  // Swapped bounds are normalised rather than producing a descending sweep.
  auto swapped = ppm::linearSpacing(5.0, 2.0, 3);
  REQUIRE(swapped.size() == 3);
  CHECK(swapped[0] == Approx(2.0));
  CHECK(swapped[2] == Approx(5.0));
}

TEST_CASE("multi-point lookahead reaches each requested distance", "[mlpp]")
{
  auto path = straightPath();
  auto distances = ppm::linearSpacing(2.0, 5.0, 3);
  auto samples = ppm::findLookaheadPoints(path, distances, 2.0);

  REQUIRE(samples.size() == 3);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    REQUIRE(samples[i].found);
    CHECK(samples[i].distance == Approx(distances[i]));
    CHECK(samples[i].point.y == Approx(0.0));  // straight path: no lateral offset
  }
  // Samples advance monotonically along the path.
  CHECK(samples[0].distance < samples[1].distance);
  CHECK(samples[1].distance < samples[2].distance);
}

TEST_CASE("multi-point lookahead agrees with the single-point helper", "[mlpp]")
{
  // The one-sweep helper must be a drop-in for N independent scans, otherwise
  // enabling the blend would silently move the targets.
  auto path = arcPath(10.0);
  std::vector<double> distances{2.0, 3.5, 5.0};
  auto samples = ppm::findLookaheadPoints(path, distances, 2.0);
  REQUIRE(samples.size() == distances.size());

  for (std::size_t i = 0; i < distances.size(); ++i) {
    auto single = ppm::findLookaheadPoint(path, distances[i], 2.0);
    CHECK(samples[i].found == single.found);
    CHECK(samples[i].point.x == Approx(single.point.x));
    CHECK(samples[i].point.y == Approx(single.point.y));
    CHECK(samples[i].segment_idx == single.segment_idx);
  }
}

TEST_CASE("multi-point lookahead honours the minimum distance floor", "[mlpp]")
{
  auto path = straightPath();
  // Requesting samples closer than min_ld must not return a nearer target.
  auto samples = ppm::findLookaheadPoints(path, {0.5, 1.0, 4.0}, 2.0);
  REQUIRE(samples.size() == 3);
  for (const auto & s : samples) {
    REQUIRE(s.found);
    CHECK(s.distance >= Approx(2.0).margin(1e-9));
  }
  CHECK(samples[2].distance == Approx(4.0));
}

TEST_CASE("multi-point lookahead clamps to the path end", "[mlpp]")
{
  // Path stops at 2.5 m, well short of the far samples. Matching the single-point
  // helper, every sample falls back to the last forward point rather than
  // reporting a miss.
  std::vector<Vec2> shortpath{{0.0, 0.0}, {1.0, 0.0}, {2.5, 0.0}};
  auto samples = ppm::findLookaheadPoints(shortpath, {2.0, 3.5, 5.0}, 2.0);

  REQUIRE(samples.size() == 3);
  for (const auto & s : samples) {
    CHECK(s.found);
  }
  CHECK(samples[1].point.x == Approx(2.5));
  CHECK(samples[2].point.x == Approx(2.5));
}

TEST_CASE("multi-point lookahead reports a miss when nothing is ahead", "[mlpp]")
{
  std::vector<Vec2> behind{{-3.0, 0.0}, {-2.0, 0.0}, {-1.0, 0.0}};
  auto samples = ppm::findLookaheadPoints(behind, {2.0, 3.5, 5.0}, 2.0);

  REQUIRE(samples.size() == 3);
  for (const auto & s : samples) {
    CHECK_FALSE(s.found);
  }
  // A blend over misses contributes nothing and steers straight.
  CHECK(ppm::blendCurvatures(samples, {0.5, 0.3, 0.2}) == Approx(0.0));
}

TEST_CASE("blended curvature with one sample equals single-point pure pursuit", "[mlpp]")
{
  // N = 1 must reproduce the existing controller exactly; this is what makes
  // enable_multi_lookahead a safe toggle.
  auto path = arcPath(10.0);
  auto samples = ppm::findLookaheadPoints(path, {4.0}, 2.0);
  REQUIRE(samples.size() == 1);
  REQUIRE(samples[0].found);

  double blended = ppm::blendCurvatures(samples, {1.0});
  double single = ppm::pursuitCurvature(ppm::findLookaheadPoint(path, 4.0, 2.0).point);
  CHECK(blended == Approx(single));
}

TEST_CASE("blended curvature normalises its weights", "[mlpp]")
{
  auto path = arcPath(10.0);
  auto samples = ppm::findLookaheadPoints(path, ppm::linearSpacing(2.0, 5.0, 3), 2.0);

  // Only the ratios matter: weights need not sum to one.
  double raw = ppm::blendCurvatures(samples, {2.0, 1.0, 1.0});
  double normalised = ppm::blendCurvatures(samples, {0.5, 0.25, 0.25});
  CHECK(raw == Approx(normalised));

  // The blend is a mean, so it sits between the extreme sample curvatures.
  double lo = ppm::pursuitCurvature(samples[0].point);
  double hi = ppm::pursuitCurvature(samples[0].point);
  for (const auto & s : samples) {
    double k = ppm::pursuitCurvature(s.point);
    lo = std::min(lo, k);
    hi = std::max(hi, k);
  }
  double blended = ppm::blendCurvatures(samples, {0.5, 0.3, 0.2});
  CHECK(blended >= Approx(lo).margin(1e-12));
  CHECK(blended <= Approx(hi).margin(1e-12));
}

TEST_CASE("blended curvature weighting shifts the command between samples", "[mlpp]")
{
  // Straight into a left turn: the near sample sees no curvature, the far one
  // sees the corner. Weighting far must command more turn-in than weighting near,
  // which is the anticipation the blend exists to provide.
  std::vector<Vec2> path;
  for (double x = 0.0; x <= 3.0; x += 0.25) {
    path.push_back({x, 0.0});
  }
  const double cx = 3.0, cy = 3.0, radius = 3.0;
  for (double theta = -M_PI / 2.0; theta <= 0.0; theta += 0.05) {
    path.push_back({cx + radius * std::cos(theta), cy + radius * std::sin(theta)});
  }

  auto samples = ppm::findLookaheadPoints(path, ppm::linearSpacing(2.0, 6.0, 3), 2.0);
  REQUIRE(samples.size() == 3);

  double near_heavy = ppm::blendCurvatures(samples, {0.8, 0.15, 0.05});
  double far_heavy = ppm::blendCurvatures(samples, {0.05, 0.15, 0.8});
  CHECK(far_heavy > near_heavy);
}

TEST_CASE("blended curvature handles degenerate weights", "[mlpp]")
{
  auto path = arcPath(10.0);
  auto samples = ppm::findLookaheadPoints(path, ppm::linearSpacing(2.0, 5.0, 3), 2.0);

  CHECK(ppm::blendCurvatures(samples, {0.0, 0.0, 0.0}) == Approx(0.0));  // no authority
  CHECK(ppm::blendCurvatures(samples, {}) == Approx(0.0));  // no weights
  CHECK(ppm::blendCurvatures({}, {0.5, 0.3, 0.2}) == Approx(0.0));  // no samples
  CHECK(ppm::blendCurvatures({}, {}) == Approx(0.0));

  // Extra weights beyond the sample count are ignored rather than skewing the mean.
  double exact = ppm::blendCurvatures(samples, {0.5, 0.3, 0.2});
  double padded = ppm::blendCurvatures(samples, {0.5, 0.3, 0.2, 0.9, 0.9});
  CHECK(exact == Approx(padded));

  // A zero weight drops its sample without changing the remaining ratios.
  auto pair = ppm::findLookaheadPoints(path, {2.0, 5.0}, 2.0);
  double dropped = ppm::blendCurvatures(pair, {1.0, 0.0});
  CHECK(dropped == Approx(ppm::pursuitCurvature(pair[0].point)));
}
