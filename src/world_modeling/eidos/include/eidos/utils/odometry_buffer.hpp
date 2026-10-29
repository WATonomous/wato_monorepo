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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>

#include <algorithm>
#include <cstddef>
#include <deque>
#include <iterator>
#include <utility>

namespace eidos
{

/**
 * @brief Time-indexed buffer of poses from an external odometry source (ROS-free).
 *
 * Used by OdometryFactor to measure the source's relative motion between two keyframe
 * timestamps. A gap between consecutive samples larger than max_gap is treated as a
 * discontinuity (e.g. a visual odometry reset), so no relative pose is reported across it.
 */
class OdometryBuffer
{
public:
  enum class Status
  {
    kOk,  ///< Relative pose computed
    kNotYetCovered,  ///< t_b is newer than the newest sample; try again later
    kOutOfRange,  ///< t_a is older than the oldest sample (or the buffer is empty)
    kGap  ///< A gap > max_gap lies inside [t_a, t_b]
  };

  OdometryBuffer(double max_gap, double duration)
  : max_gap_(max_gap)
  , duration_(duration)
  {}

  /// @brief Append a sample. Samples must be strictly increasing in time; others are ignored.
  /// @return true if the sample was stored.
  bool push(double t, const gtsam::Pose3 & pose)
  {
    if (!samples_.empty() && t <= samples_.back().first) return false;
    samples_.emplace_back(t, pose);
    while (samples_.size() > 2 && samples_.back().first - samples_.front().first > duration_) {
      samples_.pop_front();
    }
    return true;
  }

  void clear()
  {
    samples_.clear();
  }

  bool empty() const
  {
    return samples_.empty();
  }

  std::size_t size() const
  {
    return samples_.size();
  }

  double oldest() const
  {
    return samples_.front().first;
  }

  double newest() const
  {
    return samples_.back().first;
  }

  /**
   * @brief Relative pose T(t_a)^-1 * T(t_b) of the source, interpolated at both times.
   * @param t_a Start time (s).
   * @param t_b End time (s), t_b >= t_a.
   * @param out Relative pose, set only when kOk is returned.
   */
  Status relative(double t_a, double t_b, gtsam::Pose3 & out) const
  {
    if (samples_.empty() || t_a < samples_.front().first) return Status::kOutOfRange;
    if (t_b > samples_.back().first) return Status::kNotYetCovered;
    // Samples bracketing [t_a, t_b]: ia = last sample <= t_a, ib = first sample >= t_b.
    auto after_a =
      std::upper_bound(samples_.begin(), samples_.end(), t_a, [](double t, const Sample & s) { return t < s.first; });
    auto ia = static_cast<std::size_t>(std::distance(samples_.begin(), after_a)) - 1;
    auto at_b =
      std::lower_bound(samples_.begin(), samples_.end(), t_b, [](const Sample & s, double t) { return s.first < t; });
    auto ib = static_cast<std::size_t>(std::distance(samples_.begin(), at_b));
    for (std::size_t i = ia; i < ib; ++i) {
      if (samples_[i + 1].first - samples_[i].first > max_gap_) return Status::kGap;
    }
    out = interpolateAt(ia, t_a).between(interpolateAt(ib == 0 ? 0 : ib - 1, t_b));
    return Status::kOk;
  }

private:
  using Sample = std::pair<double, gtsam::Pose3>;

  /// Pose at time t, given i such that samples_[i].first <= t <= samples_[i + 1].first (or t == samples_[i].first).
  gtsam::Pose3 interpolateAt(std::size_t i, double t) const
  {
    const auto & a = samples_[i];
    if (i + 1 >= samples_.size() || t <= a.first) return a.second;
    const auto & b = samples_[i + 1];
    if (t >= b.first) return b.second;
    double alpha = (t - a.first) / (b.first - a.first);
    return a.second.interpolateRt(b.second, alpha);  // SLERP rotation, linear translation
  }

  std::deque<Sample> samples_;
  double max_gap_;
  double duration_;
};

/**
 * @brief Express a relative motion measured for a child frame (e.g. the camera rig) as the
 *        relative motion of a rigidly attached body frame.
 * @param T_body_child Extrinsic: pose of the child frame in the body frame.
 * @param rel_child Child-frame motion T_child(a)^-1 * T_child(b).
 * @return Body-frame motion T_body(a)^-1 * T_body(b).
 */
inline gtsam::Pose3 conjugateMotion(const gtsam::Pose3 & T_body_child, const gtsam::Pose3 & rel_child)
{
  return T_body_child * rel_child * T_body_child.inverse();
}

/**
 * @brief Queue of consecutive keyframe pairs waiting to be measured (ROS-free).
 *
 * Every new state forms a pair with the previous one. Pairs are consumed in order by the owner,
 * which decides per pair whether it is ready (odometry covers it and the graph has estimates for
 * both states), should wait, or should be dropped.
 */
class KeyframePairQueue
{
public:
  struct State
  {
    gtsam::Key key;
    double t;
  };

  struct Pair
  {
    State from;
    State to;
  };

  /// @brief Add a new state; pairs it with the previous state (states must arrive in time order).
  void addState(gtsam::Key key, double t)
  {
    if (has_last_ && t <= last_.t) return;
    if (has_last_) pairs_.push_back({last_, {key, t}});
    last_ = {key, t};
    has_last_ = true;
  }

  bool empty() const
  {
    return pairs_.empty();
  }

  /// @brief Oldest pending pair. Requires !empty().
  const Pair & front() const
  {
    return pairs_.front();
  }

  void pop()
  {
    pairs_.pop_front();
  }

  std::size_t size() const
  {
    return pairs_.size();
  }

  const std::deque<Pair> & pending() const
  {
    return pairs_;
  }

  void clear()
  {
    pairs_.clear();
    has_last_ = false;
  }

private:
  std::deque<Pair> pairs_;
  State last_{0, 0.0};
  bool has_last_{false};
};

}  // namespace eidos
