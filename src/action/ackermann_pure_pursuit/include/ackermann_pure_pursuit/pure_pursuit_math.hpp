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

#ifndef ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_MATH_HPP_
#define ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_MATH_HPP_

// Pure, ROS-free geometry and control helpers for the pure pursuit controller.
// Everything here operates in the vehicle reference frame: the vehicle sits at
// the origin looking down +x, +y is to the left. Keeping this ROS-free makes the
// tracking/lookahead/speed math directly unit-testable without a ROS graph.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ackermann_pure_pursuit::math
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline double norm(const Vec2 & p)
{
  return std::hypot(p.x, p.y);
}

// Wrap an angle to [-pi, pi].
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Signed cross product magnitude (z-component) of two 2D vectors.
inline double cross2(const Vec2 & a, const Vec2 & b)
{
  return a.x * b.y - a.y * b.x;
}

// --------------------------------------------------------------------------
// Tracking error
// --------------------------------------------------------------------------
struct TrackingError
{
  double cross_track{0.0};  // signed lateral distance to nearest segment (m); vehicle-left of path is positive
  double heading{0.0};  // signed angle of path tangent in vehicle frame (rad)
  std::size_t nearest_idx{0};
  bool valid{false};
};

// Cross-track and heading error of the vehicle (at origin, heading +x) relative
// to a polyline path expressed in the vehicle frame. The vehicle is projected
// onto the nearest segment (not merely the nearest vertex) so error does not
// quantize with waypoint spacing.
inline TrackingError computeTrackingError(const std::vector<Vec2> & path)
{
  TrackingError result;
  if (path.size() < 2) {
    return result;
  }

  double best_dist_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Vec2 & a = path[i];
    const Vec2 & b = path[i + 1];
    Vec2 ab{b.x - a.x, b.y - a.y};
    double len_sq = ab.x * ab.x + ab.y * ab.y;
    if (len_sq < 1e-9) {
      continue;
    }

    // Projection parameter of the origin onto segment a-b, clamped to the segment.
    double t = -(a.x * ab.x + a.y * ab.y) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    Vec2 closest{a.x + t * ab.x, a.y + t * ab.y};
    double dist_sq = closest.x * closest.x + closest.y * closest.y;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      double seg_len = std::sqrt(len_sq);
      Vec2 dir{ab.x / seg_len, ab.y / seg_len};
      // Signed cross-track: positive when the vehicle (origin) is left of the path direction.
      result.cross_track = closest.x * dir.y - closest.y * dir.x;
      result.heading = normalizeAngle(std::atan2(dir.y, dir.x));
      result.nearest_idx = i;
      result.valid = true;
    }
  }

  return result;
}

// --------------------------------------------------------------------------
// Curvature
// --------------------------------------------------------------------------

// Signed curvature (1/m) of the circle through three points. Positive = left turn.
// Returns 0 for (near-)collinear or coincident points.
inline double curvatureFromPoints(const Vec2 & a, const Vec2 & b, const Vec2 & c)
{
  Vec2 ab{b.x - a.x, b.y - a.y};
  Vec2 ac{c.x - a.x, c.y - a.y};
  double len_ab = norm(ab);
  double len_bc = std::hypot(c.x - b.x, c.y - b.y);
  double len_ca = norm(ac);
  double denom = len_ab * len_bc * len_ca;
  if (denom < 1e-6) {
    return 0.0;
  }
  // kappa = 4 * signedArea / (|AB| |BC| |CA|), signedArea = 0.5 * cross(AB, AC).
  return 2.0 * cross2(ab, ac) / denom;
}

// Estimate the signed path curvature near a given index by sampling points
// `span` vertices on either side (clamped to the path bounds).
inline double curvatureAround(const std::vector<Vec2> & path, std::size_t idx, std::size_t span)
{
  if (path.size() < 3) {
    return 0.0;
  }
  std::size_t lo = (idx > span) ? idx - span : 0;
  std::size_t hi = std::min(idx + span, path.size() - 1);
  if (hi - lo < 2) {
    lo = 0;
    hi = std::min<std::size_t>(2, path.size() - 1);
  }
  std::size_t mid = lo + (hi - lo) / 2;
  return curvatureFromPoints(path[lo], path[mid], path[hi]);
}

// Estimate the signed path curvature near `idx` from points roughly `half_arc`
// metres of arc length on either side. Unlike a fixed vertex span this is
// insensitive to waypoint density; the window clamps to the path ends.
inline double curvatureByArc(const std::vector<Vec2> & path, std::size_t idx, double half_arc)
{
  if (path.size() < 3) {
    return 0.0;
  }
  idx = std::min(idx, path.size() - 1);

  std::size_t lo = idx;
  double acc = 0.0;
  while (lo > 0 && acc < half_arc) {
    acc += std::hypot(path[lo].x - path[lo - 1].x, path[lo].y - path[lo - 1].y);
    --lo;
  }
  std::size_t hi = idx;
  acc = 0.0;
  while (hi + 1 < path.size() && acc < half_arc) {
    acc += std::hypot(path[hi + 1].x - path[hi].x, path[hi + 1].y - path[hi].y);
    ++hi;
  }
  // Need three distinct vertices; widen a degenerate window at the path ends.
  if (hi - lo < 2) {
    lo = (lo > 0) ? lo - 1 : 0;
    hi = std::min(hi + 1, path.size() - 1);
    if (hi - lo < 2) {
      return 0.0;
    }
  }
  std::size_t mid = std::clamp(idx, lo + 1, hi - 1);
  return curvatureFromPoints(path[lo], path[mid], path[hi]);
}

// --------------------------------------------------------------------------
// Lookahead point selection
// --------------------------------------------------------------------------
struct LookaheadResult
{
  Vec2 point{};
  double distance{0.0};
  std::size_t segment_idx{0};
  bool found{false};
};

// Find the point on the forward path (x > 0) at Euclidean distance `ld` from the
// vehicle. When a segment crosses the lookahead circle the exact intersection is
// interpolated so the target is continuous rather than snapping between vertices;
// this includes segments whose start lies behind the vehicle (x <= 0).
// `start_idx` skips earlier path points (e.g. anchor at the nearest segment) so a
// self-crossing path cannot match a target on an earlier lobe.
// If no forward point reaches `ld`, the last forward point is returned.
inline LookaheadResult findLookaheadPoint(
  const std::vector<Vec2> & path, double ld, double min_ld, std::size_t start_idx = 0)
{
  LookaheadResult result;
  bool have_prev = false;
  Vec2 prev{};  // previous path point; may be behind the vehicle
  Vec2 last_forward{};
  std::size_t last_idx = 0;
  bool have_last = false;

  for (std::size_t i = start_idx; i < path.size(); ++i) {
    const Vec2 & p = path[i];
    if (p.x <= 0.0) {
      prev = p;  // behind the vehicle: not a target, but still a segment start
      have_prev = true;
      continue;
    }
    double dist = norm(p);
    last_forward = p;
    last_idx = i;
    have_last = true;

    if (dist >= min_ld && dist >= ld) {
      // The segment prev -> p crosses radius ld when prev is inside the circle or
      // behind the vehicle; interpolate the crossing so the target is continuous.
      if (have_prev && (norm(prev) < ld || prev.x <= 0.0)) {
        Vec2 d{p.x - prev.x, p.y - prev.y};
        double aa = d.x * d.x + d.y * d.y;
        double bb = 2.0 * (prev.x * d.x + prev.y * d.y);
        double cc = prev.x * prev.x + prev.y * prev.y - ld * ld;
        double disc = bb * bb - 4.0 * aa * cc;
        double t = 1.0;
        if (aa > 1e-9 && disc >= 0.0) {
          double sq = std::sqrt(disc);
          double t1 = (-bb + sq) / (2.0 * aa);
          // Prefer the root inside the segment moving outward.
          t = (t1 >= 0.0 && t1 <= 1.0) ? t1 : std::clamp((-bb - sq) / (2.0 * aa), 0.0, 1.0);
        }
        Vec2 cand{prev.x + t * d.x, prev.y + t * d.y};
        // Only keep the interpolated crossing if it is ahead of the vehicle.
        result.point = (cand.x > 0.0) ? cand : p;
        result.distance = norm(result.point);
        result.segment_idx = i;
      } else {
        result.point = p;
        result.distance = dist;
        result.segment_idx = i;
      }
      result.found = true;
      return result;
    }

    prev = p;
    have_prev = true;
  }

  // No forward point reached ld: fall back to the last forward point.
  if (have_last) {
    result.point = last_forward;
    result.distance = norm(last_forward);
    result.segment_idx = last_idx;
    result.found = true;
  }
  return result;
}

// Pure pursuit curvature for a lookahead point (x, y) in the vehicle frame.
inline double pursuitCurvature(const Vec2 & lookahead)
{
  double ld_sq = lookahead.x * lookahead.x + lookahead.y * lookahead.y;
  if (ld_sq < 1e-9) {
    return 0.0;
  }
  return 2.0 * lookahead.y / ld_sq;
}

// --------------------------------------------------------------------------
// Speed / command shaping
// --------------------------------------------------------------------------

// Minimum commanded speed over the forward path (x > 0) from `start_idx` out to
// the first point at or beyond `horizon`. Taking the min (rather than sampling a
// single point) means a commanded stop inside the horizon is not skipped over;
// when the path ends inside the horizon the remaining forward speeds still bound
// the result, so an end-of-path stop is respected. Returns `fallback` when there
// are no forward points.
inline double minSpeedWithinHorizon(
  const std::vector<Vec2> & path,
  const std::vector<double> & speeds,
  double horizon,
  double fallback,
  std::size_t start_idx = 0)
{
  double result = fallback;
  bool found = false;
  const std::size_t n = std::min(path.size(), speeds.size());
  for (std::size_t i = start_idx; i < n; ++i) {
    if (path[i].x <= 0.0) {
      continue;
    }
    result = found ? std::min(result, speeds[i]) : speeds[i];
    found = true;
    if (norm(path[i]) >= horizon) {
      break;
    }
  }
  return result;
}

// Maximum speed that keeps lateral acceleration within a_lat_max on a path of
// curvature kappa: v = sqrt(a_lat_max / |kappa|), capped at v_cap.
inline double curvatureLimitedSpeed(double a_lat_max, double kappa, double v_cap)
{
  double abs_k = std::abs(kappa);
  if (abs_k < 1e-4 || a_lat_max <= 0.0) {
    return v_cap;
  }
  return std::min(v_cap, std::sqrt(a_lat_max / abs_k));
}

// Limit the change from prev to target to at most max_step in magnitude.
inline double slewLimit(double prev, double target, double max_step)
{
  if (max_step <= 0.0) {
    return target;
  }
  return std::clamp(target, prev - max_step, prev + max_step);
}

// Stanley-style cross-track correction to ADD to the (left-positive) steering
// command. Positive cte means the vehicle is left of the path, so the correction
// steers right (negative). Grows the correction at low speed.
inline double crossTrackSteerCorrection(double cte, double speed, double k_e, double k_soft)
{
  return -std::atan2(k_e * cte, k_soft + std::abs(speed));
}

}  // namespace ackermann_pure_pursuit::math

#endif  // ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_MATH_HPP_
