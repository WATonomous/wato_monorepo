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

#include "fake_planner/fake_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

// A moving frame: world position (x, y) and heading theta. Segments are authored in this local
// frame (u forward along theta, v to the left), so each simply continues from where the last ended.
struct Cursor
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// A finely-sampled polyline (parallel x/y/speed) built up before uniform resampling.
struct Path
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> speed;
};

double lerp(double a, double b, double t)
{
  return a + (b - a) * t;
}

// Append the world point for a local (u forward, v left) offset from the cursor.
void appendLocal(Path & path, const Cursor & c, double u, double v, double speed)
{
  const double cos_t = std::cos(c.theta);
  const double sin_t = std::sin(c.theta);
  path.x.push_back(c.x + u * cos_t - v * sin_t);
  path.y.push_back(c.y + u * sin_t + v * cos_t);
  path.speed.push_back(speed);
}

int subSteps(double length, double fine)
{
  return std::max(1, static_cast<int>(std::ceil(std::abs(length) / fine)));
}

// --- Primitive builders. Each appends samples for the segment interior (t in (0, L]) and moves
// the cursor to the segment end; the caller seeds the very first point. Speed ramps s0 -> s1. ---

void buildStraight(Path & path, Cursor & c, double length, double s0, double s1, double fine)
{
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    appendLocal(path, c, t, 0.0, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Shift: smooth (raised-cosine) lateral move of `lateral`, tangent to the heading at both ends.
void buildShift(Path & path, Cursor & c, double lateral, double length, double s0, double s1, double fine)
{
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    const double v = lateral * (1.0 - std::cos(kPi * t / length)) / 2.0;
    appendLocal(path, c, t, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Slalom: sinusoidal weave, amplitude tapered over the first/last half wavelength so the path
// starts and ends on the centreline tangent to the heading while the middle stays a pure sine.
void buildSlalom(
  Path & path, Cursor & c, double amp, double wavelength, double cycles, double s0, double s1, double fine)
{
  const double length = wavelength * cycles;
  const double taper = std::min(wavelength / 2.0, length / 2.0);
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    double env = amp;
    if (t < taper) {
      env = amp * (1.0 - std::cos(kPi * t / taper)) / 2.0;
    } else if (t > length - taper) {
      env = amp * (1.0 - std::cos(kPi * (length - t) / taper)) / 2.0;
    }
    const double v = env * std::sin(2.0 * kPi * t / wavelength);
    appendLocal(path, c, t, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
}

// Arc: constant-radius turn (dir_sign +1 = left/CCW, -1 = right/CW); also rotates the heading.
void buildArc(
  Path & path, Cursor & c, double radius, double angle_rad, double dir_sign, double s0, double s1, double fine)
{
  const double length = radius * angle_rad;
  const int n = subSteps(length, fine);
  for (int k = 1; k <= n; ++k) {
    const double t = length * k / n;
    const double phi = t / radius;  // angle swept so far
    const double u = radius * std::sin(phi);
    const double v = dir_sign * radius * (1.0 - std::cos(phi));
    appendLocal(path, c, u, v, lerp(s0, s1, t / length));
  }
  c.x = path.x.back();
  c.y = path.y.back();
  c.theta += dir_sign * angle_rad;
}

// Resample a fine polyline to uniform arc-length spacing (speed linearly interpolated), matching
// the uniformly-spaced points a real planner emits (trajectory_planner's interpolation_resolution).
Path resampleUniform(Path in, double spacing)
{
  if (in.x.size() < 2) {
    return in;
  }
  std::vector<double> cum(in.x.size(), 0.0);
  for (std::size_t i = 1; i < in.x.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(in.x[i] - in.x[i - 1], in.y[i] - in.y[i - 1]);
  }
  const double total = cum.back();

  Path out;
  std::size_t j = 0;
  for (double s = 0.0; s < total; s += spacing) {
    while (j + 1 < cum.size() && cum[j + 1] < s) {
      ++j;
    }
    const double seg = cum[j + 1] - cum[j];
    const double f = seg > 1e-9 ? (s - cum[j]) / seg : 0.0;
    out.x.push_back(lerp(in.x[j], in.x[j + 1], f));
    out.y.push_back(lerp(in.y[j], in.y[j + 1], f));
    out.speed.push_back(lerp(in.speed[j], in.speed[j + 1], f));
  }
  // Always finish exactly on the last authored point (unless we already landed there).
  if (out.x.empty() || std::hypot(in.x.back() - out.x.back(), in.y.back() - out.y.back()) > 1e-6) {
    out.x.push_back(in.x.back());
    out.y.push_back(in.y.back());
    out.speed.push_back(in.speed.back());
  }
  return out;
}

// Matches trajectory_planner's max_tangential_accel so the fake stop feels like the real one.
constexpr double kMaxDecel = 1.0;

// Default for the launch ramp when the maneuver does not set "ramp_up_accel"; mirrors kMaxDecel so
// the maneuver leaves rest as gently as it comes back to it.
constexpr double kDefaultRampUpAccel = 1.0;

// Start an open maneuver from a standstill: pin the first waypoint at zero speed and propagate that
// zero forwards at `accel`, so the profile asks for a gentle pull away instead of the authored speed
// from the very first point. Speed profile only, no geometry: the shipped maneuvers open with a
// straight long enough to hold the whole ramp (v^2 / 2a: 4.5 m at 3 m/s and 1 m/s^2), so lengthen
// that straight if you raise a maneuver's speed or soften its accel.
void applyLaunchRamp(Path & path, double accel)
{
  if (accel <= 0.0 || path.x.size() < 2) {
    return;
  }
  path.speed.front() = 0.0;
  for (std::size_t i = 1; i < path.speed.size(); ++i) {
    const double seg = std::hypot(path.x[i] - path.x[i - 1], path.y[i] - path.y[i - 1]);
    const double v_max = std::sqrt(path.speed[i - 1] * path.speed[i - 1] + 2.0 * accel * seg);
    path.speed[i] = std::min(path.speed[i], v_max);
  }
}

// How far past the last authored point the zero-speed pad extends. Pure pursuit only steers to
// points *ahead* of the vehicle and holds its last command when it runs out, so the path has to
// stay populated past the stop line. Comfortably longer than the controller's max lookahead.
constexpr double kStopPadM = 10.0;

// Terminate an open maneuver in a standstill: extend the final heading with zero-speed points, then
// propagate that zero backwards at kMaxDecel so the speed profile brakes into it.
void appendStopPad(Path & path, double spacing)
{
  if (path.x.size() < 2) {
    return;
  }
  const double dx = path.x.back() - path.x[path.x.size() - 2];
  const double dy = path.y.back() - path.y[path.y.size() - 2];
  const double norm = std::hypot(dx, dy);
  if (norm < 1e-9) {
    return;
  }

  const double end_x = path.x.back();
  const double end_y = path.y.back();
  const int pad_points = static_cast<int>(std::ceil(kStopPadM / spacing));
  for (int k = 1; k <= pad_points; ++k) {
    const double t = spacing * k;
    path.x.push_back(end_x + dx / norm * t);
    path.y.push_back(end_y + dy / norm * t);
    path.speed.push_back(0.0);
  }
  // The last authored point is the stop line itself.
  path.speed.back() = 0.0;
  path.speed[path.speed.size() - pad_points - 1] = 0.0;

  for (int i = static_cast<int>(path.speed.size()) - 2; i >= 0; --i) {
    const double seg = std::hypot(path.x[i + 1] - path.x[i], path.y[i + 1] - path.y[i]);
    const double v_max = std::sqrt(path.speed[i + 1] * path.speed[i + 1] + 2.0 * kMaxDecel * seg);
    path.speed[i] = std::min(path.speed[i], v_max);
  }
}

}  // namespace

namespace fake_planner
{

FakePlannerCore::FakePlannerCore(const FakePlannerConfig & config)
: config_(config)
{}

bool FakePlannerCore::loadManeuver(const std::string & path, std::string & error)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    error = "cannot open file";
    return false;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return loadManeuverJson(buffer.str(), error);
}

bool FakePlannerCore::loadManeuverJson(const std::string & json_text, std::string & error)
{
  nlohmann::json doc;
  try {
    doc = nlohmann::json::parse(json_text);
  } catch (const std::exception & e) {
    error = std::string("invalid JSON: ") + e.what();
    return false;
  }
  if (!doc.is_object()) {
    error = "top-level JSON must be an object";
    return false;
  }

  const double spacing = doc.value("sample_spacing_m", 1.0);
  const double default_speed = doc.value("default_speed", 2.0);
  const double ramp_up_accel = doc.value("ramp_up_accel", kDefaultRampUpAccel);
  closed_ = doc.value("closed", false);
  has_absolute_start_ = false;
  if (spacing <= 0.0) {
    error = "sample_spacing_m must be > 0";
    return false;
  }
  if (ramp_up_accel < 0.0) {
    error = "ramp_up_accel must be >= 0 (0 disables the launch ramp)";
    return false;
  }
  if (!doc.contains("segments") || !doc["segments"].is_array() || doc["segments"].empty()) {
    error = "missing or empty 'segments' array";
    return false;
  }

  // Walk the segments as a moving frame, sampling each finely, then resample to uniform spacing.
  const double fine = std::min(0.05, spacing);
  Path fine_path;
  Cursor cursor;

  // Optional absolute start pose, which also marks the maneuver as fixed geometry.
  if (doc.contains("start")) {
    const nlohmann::json & start = doc["start"];
    if (!start.is_object()) {
      error = "'start' must be an object with x/y/yaw";
      return false;
    }
    cursor.x = start.value("x", 0.0);
    cursor.y = start.value("y", 0.0);
    cursor.theta = start.value("yaw", 0.0);
    has_absolute_start_ = true;
  }

  double current_speed = default_speed;  // carried between segments for speed continuity

  std::size_t idx = 0;
  for (const auto & seg : doc["segments"]) {
    ++idx;
    if (!seg.is_object() || seg.size() != 1) {
      error = "segment " + std::to_string(idx) + ": expected a single {type: {...}} object";
      return false;
    }
    const auto it = seg.begin();
    const std::string type = it.key();
    const nlohmann::json & p = it.value();
    if (!p.is_object()) {
      error = "segment " + std::to_string(idx) + " (" + type + "): parameters must be an object";
      return false;
    }

    const double s0 = p.value("speed", current_speed);
    const double s1 = p.value("end_speed", s0);

    // Seed the very first path point at the start pose and speed.
    if (fine_path.x.empty()) {
      appendLocal(fine_path, cursor, 0.0, 0.0, s0);
    }

    try {
      if (type == "straight" || type == "dwell") {
        buildStraight(fine_path, cursor, p.at("length").get<double>(), s0, s1, fine);
      } else if (type == "shift") {
        buildShift(fine_path, cursor, p.at("lateral").get<double>(), p.at("length").get<double>(), s0, s1, fine);
      } else if (type == "slalom") {
        buildSlalom(
          fine_path,
          cursor,
          p.at("amplitude").get<double>(),
          p.at("wavelength").get<double>(),
          p.at("cycles").get<double>(),
          s0,
          s1,
          fine);
      } else if (type == "arc") {
        const std::string dir = p.value("dir", std::string("left"));
        if (dir != "left" && dir != "right") {
          error = "segment " + std::to_string(idx) + " (arc): dir must be 'left' or 'right'";
          return false;
        }
        const double dir_sign = (dir == "left") ? 1.0 : -1.0;
        buildArc(
          fine_path,
          cursor,
          p.at("radius").get<double>(),
          p.at("angle").get<double>() * kPi / 180.0,
          dir_sign,
          s0,
          s1,
          fine);
      } else {
        error = "segment " + std::to_string(idx) + ": unknown type '" + type + "'";
        return false;
      }
    } catch (const std::exception & e) {
      error = "segment " + std::to_string(idx) + " (" + type + "): " + e.what();
      return false;
    }

    current_speed = s1;
  }

  Path resampled = resampleUniform(std::move(fine_path), spacing);
  // A closed circuit has no end to stop at and no start to pull away from -- the window wraps at
  // the seam, so a ramp there would make the car brake to a crawl once a lap.
  if (!closed_) {
    applyLaunchRamp(resampled, ramp_up_accel);
    appendStopPad(resampled, spacing);
  }
  wp_x_ = resampled.x;
  wp_y_ = resampled.y;
  wp_speed_ = resampled.speed;

  // The stop line is one past the last waypoint that still asks for speed. Found from the profile
  // rather than from kStopPadM so it stays right whatever the pad does.
  cum_s_.assign(wp_x_.size(), 0.0);
  for (std::size_t i = 1; i < wp_x_.size(); ++i) {
    cum_s_[i] = cum_s_[i - 1] + std::hypot(wp_x_[i] - wp_x_[i - 1], wp_y_[i] - wp_y_[i - 1]);
  }
  stop_index_ = wp_x_.empty() ? 0 : wp_x_.size() - 1;
  for (std::size_t i = wp_speed_.size(); i-- > 0;) {
    if (wp_speed_[i] > 0.0) {
      stop_index_ = std::min(i + 1, wp_x_.size() - 1);
      break;
    }
  }
  return true;
}

double FakePlannerCore::distanceToEnd() const
{
  if (closed_ || !ready_ || cum_s_.empty() || window_start_ >= cum_s_.size()) {
    return std::numeric_limits<double>::infinity();
  }
  // Negative once the vehicle is into the pad past the stop line -- that is arrived, so clamp.
  return std::max(0.0, cum_s_[stop_index_] - cum_s_[window_start_]);
}

void FakePlannerCore::anchor(double x, double y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // Transform every waypoint up front so publishing only has to restamp the sliced window.
  std::vector<double> xs(wp_x_.size());
  std::vector<double> ys(wp_x_.size());
  for (std::size_t i = 0; i < wp_x_.size(); ++i) {
    xs[i] = x + wp_x_[i] * c - wp_y_[i] * s;
    ys[i] = y + wp_x_[i] * s + wp_y_[i] * c;
  }

  wato_trajectory_msgs::msg::Trajectory traj;
  traj.header.frame_id = config_.frame_id;
  traj.points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    wato_trajectory_msgs::msg::TrajectoryPoint pt;
    pt.pose.position.x = xs[i];
    pt.pose.position.y = ys[i];
    pt.pose.position.z = 0.0;

    // Yaw from consecutive points (last point reuses the previous heading).
    double point_yaw = 0.0;
    if (i + 1 < xs.size()) {
      point_yaw = std::atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i]);
    } else if (i > 0) {
      point_yaw = std::atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]);
    }
    pt.pose.orientation.z = std::sin(point_yaw / 2.0);
    pt.pose.orientation.w = std::cos(point_yaw / 2.0);

    pt.max_speed = wp_speed_[i];
    traj.points.push_back(pt);
  }

  full_traj_ = traj;
  // Until the first pose slides the window, the whole maneuver is what gets published.
  window_ = full_traj_;
  window_start_ = 0;
  ready_ = true;
}

void FakePlannerCore::rewind()
{
  window_start_ = 0;
}

void FakePlannerCore::clear()
{
  full_traj_ = wato_trajectory_msgs::msg::Trajectory();
  window_ = wato_trajectory_msgs::msg::Trajectory();
  window_start_ = 0;
  ready_ = false;
}

std::size_t FakePlannerCore::nearestIndex(double veh_x, double veh_y) const
{
  const std::size_t n = full_traj_.points.size();
  // Search a bounded stretch ahead of the last match rather than the whole path: a slalom or a
  // closed lap passes near its own earlier points, and a global search would jump back to them.
  constexpr std::size_t kSearchAhead = 400;
  const std::size_t span = std::min(kSearchAhead, n);

  std::size_t best = window_start_;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t k = 0; k < span; ++k) {
    const std::size_t i = closed_ ? (window_start_ + k) % n : std::min(window_start_ + k, n - 1);
    const auto & p = full_traj_.points[i].pose.position;
    const double d = std::hypot(p.x - veh_x, p.y - veh_y);
    if (d < best_dist) {
      best_dist = d;
      best = i;
    }
  }
  return best;
}

// Slice the published horizon out of the full maneuver: trail_m behind the vehicle through
// horizon_m ahead. On a closed circuit the window wraps and the car laps; on an open one it runs
// into the zero-speed stop pad and holds there.
void FakePlannerCore::updateWindow(double veh_x, double veh_y)
{
  const std::size_t n = full_traj_.points.size();
  if (n == 0) {
    return;
  }

  const std::size_t nearest = nearestIndex(veh_x, veh_y);
  window_start_ = nearest;

  wato_trajectory_msgs::msg::Trajectory window;
  window.header = full_traj_.header;

  // Walk backwards from the nearest point to lay down the trailing stub.
  std::size_t first = nearest;
  double trailed = 0.0;
  while (trailed < config_.trail_m) {
    const std::size_t prev = (first == 0) ? (closed_ ? n - 1 : 0) : first - 1;
    if (prev == first) {
      break;  // open path, already at the start
    }
    const auto & a = full_traj_.points[prev].pose.position;
    const auto & b = full_traj_.points[first].pose.position;
    trailed += std::hypot(b.x - a.x, b.y - a.y);
    first = prev;
  }

  double covered = 0.0;
  std::size_t i = first;
  for (std::size_t k = 0; k < n; ++k) {
    window.points.push_back(full_traj_.points[i]);
    const std::size_t next = (i + 1 == n) ? (closed_ ? 0 : i) : i + 1;
    if (next == i) {
      break;  // open path, ran off the end
    }
    const auto & a = full_traj_.points[i].pose.position;
    const auto & b = full_traj_.points[next].pose.position;
    covered += std::hypot(b.x - a.x, b.y - a.y);
    i = next;
    if (covered >= trailed + config_.horizon_m) {
      break;
    }
  }

  window_ = window;
}

}  // namespace fake_planner
