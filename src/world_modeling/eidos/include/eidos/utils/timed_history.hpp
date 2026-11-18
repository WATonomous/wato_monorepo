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

#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <optional>
#include <utility>

namespace eidos
{

/**
 * @brief Short time-ordered history of samples with linear interpolation (ROS-free, not thread-safe).
 *
 * Used to read two sensors at one common time, e.g. the INS heading and the GPS position at a
 * keyframe's time.
 */
template <typename T>
class TimedHistory
{
public:
  explicit TimedHistory(double duration)
  : duration_(duration)
  {}

  /// @brief Append a sample. Samples not newer than the newest are ignored.
  void push(double t, const T & value)
  {
    if (!samples_.empty() && t <= samples_.back().first) return;
    samples_.emplace_back(t, value);
    while (samples_.size() > 2 && samples_.back().first - samples_.front().first > duration_) samples_.pop_front();
  }

  /// @brief The two samples around t and the weight of the second, or nullopt if t is not covered.
  std::optional<std::pair<std::pair<T, T>, double>> bracket(double t) const
  {
    if (samples_.empty() || t < samples_.front().first || t > samples_.back().first) return std::nullopt;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      if (samples_[i].first >= t) {
        const auto & a = samples_[i - 1];
        const auto & b = samples_[i];
        return std::make_pair(std::make_pair(a.second, b.second), (t - a.first) / (b.first - a.first));
      }
    }
    return std::make_pair(std::make_pair(samples_.back().second, samples_.back().second), 0.0);
  }

private:
  std::deque<std::pair<double, T>> samples_;
  double duration_;
};

/// @brief Point interpolated linearly at t, or nullopt if t is not covered.
inline std::optional<Eigen::Vector3d> interpolatePoint(const TimedHistory<Eigen::Vector3d> & history, double t)
{
  const auto b = history.bracket(t);
  if (!b) return std::nullopt;
  return (1.0 - b->second) * b->first.first + b->second * b->first.second;
}

/// @brief Angle (rad) interpolated along the shorter way at t, or nullopt if t is not covered.
inline std::optional<double> interpolateAngle(const TimedHistory<double> & history, double t)
{
  const auto b = history.bracket(t);
  if (!b) return std::nullopt;
  const double a = b->first.first;
  const double diff = std::remainder(b->first.second - a, 2.0 * M_PI);
  return std::remainder(a + b->second * diff, 2.0 * M_PI);
}

}  // namespace eidos
