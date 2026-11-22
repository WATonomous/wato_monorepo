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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <vector>

namespace wato::perception::bevfusion
{

/**
 * @brief Azimuth of a camera's viewing direction in the LiDAR frame, in degrees (atan2(y, x)).
 * @param camera_to_lidar Row-major 4x4 transform from the camera (optical) frame to the LiDAR frame.
 *        The camera looks along its +z axis, so the direction is the rotation's third column.
 */
inline double cameraAzimuthDeg(const float * camera_to_lidar)
{
  return std::atan2(static_cast<double>(camera_to_lidar[6]), static_cast<double>(camera_to_lidar[2])) * 180.0 / M_PI;
}

/// Which point of the sweep the scan's header stamp is the time of
enum class StampPosition
{
  kEnd,  ///< The last point (Velodyne driver with timestamp_first_packet: false, Eve's setting)
  kStart  ///< The first point (timestamp_first_packet: true, and drivers that stamp the start of the frame)
};

/**
 * @brief The LiDAR's rotation period, learned from the spacing of its scan stamps (one scan is one turn).
 * @details The median of the last 20 stamp steps, once 5 are known. A step of 0 or less (a bag restarting)
 *          starts over, and steps of 1 s or more (a pause) are ignored. A dropped scan is one double-length
 *          step and does not move the median. It uses stamps only, so it is the same live and at any bag
 *          playback rate.
 */
class RotationPeriodEstimator
{
public:
  /// Add the stamp of the newest scan
  void addStamp(int64_t stamp_ns)
  {
    if (last_stamp_ns_) {
      const int64_t step = stamp_ns - *last_stamp_ns_;
      if (step <= 0) {
        steps_.clear();
      } else if (step < kMaxStepNs) {
        steps_.push_back(step);
        if (steps_.size() > kWindow) {
          steps_.pop_front();
        }
      }
    }
    last_stamp_ns_ = stamp_ns;
  }

  /// Learned rotation period, or nothing until 5 steps are known
  std::optional<double> periodNs() const
  {
    if (steps_.size() < kMinSteps) {
      return std::nullopt;
    }
    std::vector<int64_t> sorted(steps_.begin(), steps_.end());
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2), sorted.end());
    return static_cast<double>(sorted[sorted.size() / 2]);
  }

  /// Forget everything learned
  void reset()
  {
    steps_.clear();
    last_stamp_ns_.reset();
  }

private:
  static constexpr size_t kWindow = 20;
  static constexpr size_t kMinSteps = 5;
  static constexpr int64_t kMaxStepNs = 1'000'000'000;
  std::deque<int64_t> steps_;
  std::optional<int64_t> last_stamp_ns_;
};

/// Wrap an angle in degrees to (-180, 180]
inline double wrapDeg(double a)
{
  a = std::fmod(a + 180.0, 360.0);
  if (a <= 0.0) {
    a += 360.0;
  }
  return a - 180.0;
}

/**
 * @brief When a spinning LiDAR's beam swept each camera's viewing direction during one scan.
 *
 * Velodyne scans store points in firing order. On Eve they are stamped with the last packet
 * (timestamp_first_packet: false, StampPosition::kEnd), so the azimuth of the last points is where the
 * beam was at the stamp. A direction the beam passed earlier was swept the angle it had still to travel,
 * divided by the rotation speed, before the stamp:
 *
 *   target = stamp - angle(camera azimuth -> end azimuth, in the rotation direction) / 360 * period
 *
 * A scan stamped with its first point (StampPosition::kStart) is timed from the azimuth of its first points:
 *
 *   target = stamp + angle(start azimuth -> camera azimuth, in the rotation direction) / 360 * period
 *
 * The rotation direction and that the cloud is one sweep are checked on an evenly spaced sample of
 * points: the net rotation must be about one turn, no step between neighbouring samples may exceed
 * 90 deg, and the sample must never fall back more than 20 deg behind the furthest angle it has reached
 * (per-laser azimuth offsets make single steps go either way; recorded scans fall back at most 7 deg).
 * Clouds that are not one ordered sweep (merged from several LiDARs, reordered or ring-major points, a
 * partial scan) return no result, and the caller should fall back to the scan stamp. A camera within a
 * few degrees of where the sweep starts and ends (it drifts from scan to scan when the driver's cut_angle
 * is unset) can be given either of the two passes, about one period apart; its image straddles the cut.
 *
 * @param data Point buffer (PointCloud2 data)
 * @param num_points Number of points in the buffer
 * @param point_step Bytes per point
 * @param x_offset Byte offset of the FLOAT32 x field within a point
 * @param y_offset Byte offset of the FLOAT32 y field within a point
 * @param stamp_ns Scan stamp, the time of the last point (or of the first, see stamp_position)
 * @param camera_azimuths_deg Azimuth of each camera's viewing direction in the LiDAR frame
 * @param rotation_period_ns Time for one full rotation (60 s / RPM; see RotationPeriodEstimator)
 * @param stamp_position Which point the stamp is the time of
 * @param samples Points sampled to check the sweep (cost is O(samples); at least 128 are used)
 * @return Target time per camera, or nullopt if the cloud is not one ordered sweep (or has under 128 points)
 */
inline std::optional<std::vector<int64_t>> sweepTargets(
  const uint8_t * data,
  size_t num_points,
  size_t point_step,
  size_t x_offset,
  size_t y_offset,
  int64_t stamp_ns,
  const std::vector<double> & camera_azimuths_deg,
  double rotation_period_ns,
  StampPosition stamp_position = StampPosition::kEnd,
  size_t samples = 256)
{
  constexpr size_t kMinSamples = 128;
  constexpr double kMaxStepDeg = 90.0;
  constexpr double kMaxFallBackDeg = 20.0;
  if (
    !data || num_points < kMinSamples || point_step < std::max(x_offset, y_offset) + sizeof(float) ||
    !std::isfinite(rotation_period_ns) || rotation_period_ns <= 0.0)
  {
    return std::nullopt;
  }
  const auto azimuth = [&](size_t i, double & out) {
    float x;
    float y;
    std::memcpy(&x, data + i * point_step + x_offset, sizeof(float));
    std::memcpy(&y, data + i * point_step + y_offset, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || (x == 0.0f && y == 0.0f)) {
      return false;
    }
    out = std::atan2(static_cast<double>(y), static_cast<double>(x)) * 180.0 / M_PI;
    return true;
  };

  // Net rotation over the scan from evenly spaced samples (steps between them must be < 180 deg to unwrap)
  samples = std::clamp<size_t>(samples, kMinSamples, num_points);
  std::vector<double> steps;
  steps.reserve(samples);
  double total = 0.0;
  double prev = 0.0;
  bool have_prev = false;
  for (size_t s = 0; s < samples; ++s) {
    double a;
    if (!azimuth(s * (num_points - 1) / (samples - 1), a)) {
      continue;
    }
    if (have_prev) {
      const double step = wrapDeg(a - prev);
      if (std::fabs(step) > kMaxStepDeg) {
        return std::nullopt;  // not in firing order (or a blind sector wider than 90 deg)
      }
      steps.push_back(step);
      total += step;
    }
    prev = a;
    have_prev = true;
  }
  if (std::fabs(total) < 300.0 || std::fabs(total) > 420.0) {
    return std::nullopt;
  }
  const double direction = total < 0.0 ? -1.0 : 1.0;
  double progress = 0.0;
  double furthest = 0.0;
  for (const double step : steps) {
    progress += step * direction;
    furthest = std::max(furthest, progress);
    if (furthest - progress > kMaxFallBackDeg) {
      return std::nullopt;  // turns back: not in firing order
    }
  }

  // Azimuth at the stamp: circular mean of the last (or first) valid points; per-laser azimuth offsets add a
  // few degrees of noise
  constexpr size_t kEdgePoints = 32;
  const bool at_start = stamp_position == StampPosition::kStart;
  double sx = 0.0;
  double sy = 0.0;
  size_t used = 0;
  for (size_t k = 0; k < num_points && used < kEdgePoints; ++k) {
    double a;
    if (azimuth(at_start ? k : num_points - 1 - k, a)) {
      sx += std::cos(a * M_PI / 180.0);
      sy += std::sin(a * M_PI / 180.0);
      ++used;
    }
  }
  if (used == 0) {
    return std::nullopt;
  }
  const double stamp_azimuth = std::atan2(sy, sx) * 180.0 / M_PI;

  std::vector<int64_t> targets;
  targets.reserve(camera_azimuths_deg.size());
  for (const double cam : camera_azimuths_deg) {
    if (!std::isfinite(cam)) {
      targets.push_back(stamp_ns);  // unknown direction: use the scan stamp
      continue;
    }
    // Angle between the beam at the stamp and the camera, travelling forwards in time from the earlier one
    double angle = std::fmod((at_start ? cam - stamp_azimuth : stamp_azimuth - cam) * direction, 360.0);
    if (angle < 0.0) {
      angle += 360.0;
    }
    const int64_t offset = static_cast<int64_t>(std::llround(angle / 360.0 * rotation_period_ns));
    targets.push_back(at_start ? stamp_ns + offset : stamp_ns - offset);
  }
  return targets;
}

}  // namespace wato::perception::bevfusion
