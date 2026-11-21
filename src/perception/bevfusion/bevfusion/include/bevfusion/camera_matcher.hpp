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
#include <cstdlib>
#include <deque>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace wato::perception::bevfusion
{

/**
 * @brief Settings for CameraMatcher. All times are in nanoseconds.
 */
struct CameraMatcherConfig
{
  size_t num_cameras = 6;
  /// Longest wait after a scan arrives before using the best frame already received. < 0 waits until
  /// the choice is certain, with a 1 s safety limit (and at most max_pending_scans scans waiting).
  int64_t max_wait_ns = 60'000'000;
  /// Largest |frame stamp - target| a frame may have and still be used. A scan with a camera outside
  /// this is skipped.
  int64_t tolerance_ns = 450'000'000;
  /// Decide early on the newest frame before the target when it is within early_fraction of the time to
  /// the earliest moment the camera's next frame can be stamped. That moment is bounded from the newest two
  /// frames and the camera's shortest frame interval (less interval_jitter), learned from its last 200
  /// intervals once at least 20 are known. Values up to 0.5 never change which frame is picked, unless a
  /// frame interval is more than interval_jitter shorter than that minimum for two frames in a row (e.g. two
  /// consecutive frames stamped late), or a camera's frame rate rises after it ran slower for its whole window.
  double early_fraction = 0.5;
  /// How much shorter than its recent minimum a camera's frame interval may be (stamp jitter) while the
  /// early rule stays exact. In the drive bag single intervals range from 45.9 to 53.9 ms (minimum about 49 ms).
  double interval_jitter = 0.1;
  /// Frames kept per camera. On the NITROS path each kept frame holds one GPU buffer of the producer's pool.
  size_t frames_per_camera = 6;
  /// Scans that can wait for their frames at the same time. Beyond this the oldest is decided at once
  /// with the frames it has (as if its wait had run out).
  size_t max_pending_scans = 4;
  /// A stamp this far behind the newest one of the same stream (e.g. a bag restarting) clears everything.
  int64_t time_jump_ns = 200'000'000;
};

/**
 * @brief Running totals kept by CameraMatcher, for diagnostics.
 */
struct CameraMatcherCounters
{
  uint64_t scans = 0;  ///< Scans added
  uint64_t matched = 0;  ///< Scans that produced a matched set
  uint64_t dropped_out_of_tolerance = 0;  ///< A camera's chosen frame was further than the tolerance from its target
  uint64_t dropped_no_frame = 0;  ///< When the wait ran out, some camera had no frame at or before its target
  uint64_t dropped_overflow = 0;  ///< Too many scans waiting, and the oldest could not be matched with what it had
  uint64_t resets = 0;  ///< Times a backward time jump cleared all state
};

/**
 * @brief Matches each LiDAR scan with one frame per camera, LiDAR-anchored.
 *
 * Every scan carries one target time per camera, normally the moment the LiDAR swept that camera's
 * viewing direction (see lidar_sweep_timing.hpp), or the scan stamp. For each camera the frame whose
 * stamp is nearest its target is used. A camera is decided as soon as that choice is certain:
 *   - a frame stamped after the target has arrived (one camera's frames arrive in stamp order, so no
 *     later frame can be nearer), or
 *   - the newest frame before the target is close enough that the camera's next frame cannot be nearer
 *     (see CameraMatcherConfig::early_fraction).
 * A camera's choice is final once made: later frames and evictions do not change it. If the choice is
 * not certain max_wait_ns after the scan arrived, the newest frame before the target is used, and a scan
 * with any chosen frame further than the tolerance from its target is skipped.
 *
 * Frames older than the frames_per_camera kept are gone: a scan whose target falls before them uses
 * the oldest kept frame (if within the tolerance). This only happens when a scan arrives far later than
 * the camera frames around its targets.
 *
 * All times are passed in, so the same code runs live and in tests. Not thread-safe: call it from one
 * thread or under one lock. Frames and scans are held by value; use shared pointers so that dropping
 * them (on eviction or clear()) releases the underlying buffers. FrameT must be copyable (a frame can
 * be used by several scans); ScanT only needs to be movable.
 *
 * @tparam FrameT Per-camera frame payload (e.g. a shared pointer to an image message)
 * @tparam ScanT Scan payload (e.g. a shared pointer to a point cloud)
 */
template <typename FrameT, typename ScanT>
class CameraMatcher
{
public:
  /// A camera frame as held by the matcher
  struct Frame
  {
    int64_t stamp_ns = 0;
    int64_t arrival_ns = 0;
    FrameT data{};
  };

  /// One scan with the frame chosen for every camera
  struct MatchedSet
  {
    int64_t scan_stamp_ns = 0;
    int64_t scan_arrival_ns = 0;
    int64_t decided_ns = 0;  ///< Time the set became complete
    ScanT scan{};
    std::vector<int64_t> targets_ns;  ///< Target time per camera, in camera order
    std::vector<Frame> frames;  ///< Chosen frame per camera, in camera order
    int64_t max_gap_ns = 0;  ///< Largest |frame stamp - target| in the set
  };

  explicit CameraMatcher(const CameraMatcherConfig & config)
  : config_(config)
  , cameras_(config.num_cameras)
  {}

  const CameraMatcherConfig & config() const
  {
    return config_;
  }

  const CameraMatcherCounters & counters() const
  {
    return counters_;
  }

  /**
   * @brief Add a camera frame. Completed sets are appended to out.
   * @param camera Camera index in [0, num_cameras); other values are ignored
   * @param stamp_ns Frame capture stamp
   * @param data Frame payload
   * @param now_ns Current time (arrival time of the frame)
   * @param out Receives any sets completed by this frame
   */
  void addFrame(size_t camera, int64_t stamp_ns, FrameT data, int64_t now_ns, std::vector<MatchedSet> & out)
  {
    if (camera >= cameras_.size()) {
      return;
    }
    auto & cam = cameras_[camera];
    if (!cam.frames.empty() && cam.frames.back().stamp_ns - stamp_ns > config_.time_jump_ns) {
      clear();
      ++counters_.resets;
    }
    // Keep frames sorted by stamp; arrivals are almost always already in order
    auto it = cam.frames.end();
    while (it != cam.frames.begin() && std::prev(it)->stamp_ns > stamp_ns) {
      --it;
    }
    if (it != cam.frames.begin() && std::prev(it)->stamp_ns == stamp_ns) {
      return;  // duplicate
    }
    if (it == cam.frames.begin() && cam.frames.size() >= framesPerCamera()) {
      return;  // older than every kept frame of a full buffer: it would be evicted at once
    }
    it = cam.frames.insert(it, Frame{stamp_ns, now_ns, std::move(data)});
    recordIntervals(cam, static_cast<size_t>(std::distance(cam.frames.begin(), it)));
    while (cam.frames.size() > framesPerCamera()) {
      cam.frames.pop_front();
    }
    evaluate(now_ns, out);
  }

  /**
   * @brief Add a LiDAR scan. Completed sets are appended to out.
   * @param stamp_ns Scan stamp
   * @param scan Scan payload
   * @param targets_ns Target time per camera; if its size is not num_cameras, the scan stamp is used for all
   * @param now_ns Current time (arrival time of the scan)
   * @param out Receives any sets completed by this scan
   */
  void addScan(
    int64_t stamp_ns, ScanT scan, std::vector<int64_t> targets_ns, int64_t now_ns, std::vector<MatchedSet> & out)
  {
    if (last_scan_stamp_ns_ && *last_scan_stamp_ns_ - stamp_ns > config_.time_jump_ns) {
      clear();
      ++counters_.resets;
    }
    last_scan_stamp_ns_ = stamp_ns;
    ++counters_.scans;
    if (targets_ns.size() != cameras_.size()) {
      targets_ns.assign(cameras_.size(), stamp_ns);
    }
    Pending p;
    p.stamp_ns = stamp_ns;
    p.arrival_ns = now_ns;
    p.scan = std::move(scan);
    p.targets_ns = std::move(targets_ns);
    p.chosen.resize(cameras_.size());
    pending_.push_back(std::move(p));
    // Too many waiting: decide the oldest at once with what it has, as if its wait had run out
    // (first, so that sets come out in scan order)
    while (pending_.size() > std::max<size_t>(config_.max_pending_scans, 1)) {
      if (complete(pending_.front(), now_ns, true, out) != Outcome::kEmitted) {
        ++counters_.dropped_overflow;
      }
      pending_.pop_front();
    }
    evaluate(now_ns, out);
  }

  /**
   * @brief Re-evaluate waiting scans at the given time (call when the deadline from nextDeadline() is reached).
   */
  void tick(int64_t now_ns, std::vector<MatchedSet> & out)
  {
    evaluate(now_ns, out);
  }

  /**
   * @brief Earliest deadline of the waiting scans: the time at which one can complete or be dropped
   *        without new messages. May be at or before the current time if tick() is overdue.
   */
  std::optional<int64_t> nextDeadline() const
  {
    std::optional<int64_t> best;
    for (const auto & p : pending_) {
      const int64_t d = deadline(p);
      if (!best || d < *best) {
        best = d;
      }
    }
    return best;
  }

  /// Number of scans waiting for frames
  size_t pendingScans() const
  {
    return pending_.size();
  }

  /// Number of frames held for a camera
  size_t framesHeld(size_t camera) const
  {
    return camera < cameras_.size() ? cameras_[camera].frames.size() : 0;
  }

  /**
   * @brief Drop all frames and waiting scans (releases their payloads). Counters are kept.
   */
  void clear()
  {
    for (auto & cam : cameras_) {
      cam.frames.clear();
      cam.intervals.clear();
    }
    pending_.clear();
    last_scan_stamp_ns_.reset();
  }

private:
  static constexpr int64_t kUnlimitedWaitLimitNs = 1'000'000'000;
  /// Frame intervals of 1 s or more are not a frame rate (a stall or a pause) and are not learned
  static constexpr int64_t kMaxIntervalNs = 1'000'000'000;
  /// Intervals remembered per camera for the early rule (10 s at 20 Hz)
  static constexpr size_t kIntervalWindow = 200;
  /// Intervals a camera must have shown (since start or a reset) before the early rule is used
  static constexpr size_t kMinIntervalsForEarly = 20;

  struct Camera
  {
    std::deque<Frame> frames;  // sorted by stamp
    std::deque<int64_t> intervals;  // recent stamp differences between neighbouring frames
  };

  enum class Outcome
  {
    kWaiting,
    kEmitted,
    kNoFrame,
    kOutOfTolerance
  };

  struct Pending
  {
    int64_t stamp_ns = 0;
    int64_t arrival_ns = 0;
    ScanT scan{};
    std::vector<int64_t> targets_ns;
    std::vector<std::optional<Frame>> chosen;  // per camera, final once set
  };

  size_t framesPerCamera() const
  {
    return std::max<size_t>(config_.frames_per_camera, 1);
  }

  int64_t deadline(const Pending & p) const
  {
    const int64_t wait = config_.max_wait_ns >= 0 ? config_.max_wait_ns : kUnlimitedWaitLimitNs;
    return p.arrival_ns > std::numeric_limits<int64_t>::max() - wait ? std::numeric_limits<int64_t>::max()
                                                                     : p.arrival_ns + wait;
  }

  static void recordInterval(Camera & cam, int64_t dt)
  {
    if (dt <= 0 || dt >= kMaxIntervalNs) {
      return;
    }
    cam.intervals.push_back(dt);
    if (cam.intervals.size() > kIntervalWindow) {
      cam.intervals.pop_front();
    }
  }

  /// Learn the intervals between a newly inserted frame and its neighbours
  static void recordIntervals(Camera & cam, size_t index)
  {
    if (index > 0) {
      recordInterval(cam, cam.frames[index].stamp_ns - cam.frames[index - 1].stamp_ns);
    }
    if (index + 1 < cam.frames.size()) {
      recordInterval(cam, cam.frames[index + 1].stamp_ns - cam.frames[index].stamp_ns);
    }
  }

  /**
   * @brief Shortest recent frame interval, if it can be trusted: at least kMinIntervalsForEarly intervals must
   *        be known, and the shortest must be confirmed by a second within interval_jitter of it, so a single
   *        interval spanning a dropped frame is never taken as the frame interval.
   */
  std::optional<int64_t> confirmedMinInterval(const Camera & cam) const
  {
    if (cam.intervals.size() < kMinIntervalsForEarly) {
      return std::nullopt;
    }
    int64_t first = std::numeric_limits<int64_t>::max();
    int64_t second = std::numeric_limits<int64_t>::max();
    for (const int64_t dt : cam.intervals) {
      if (dt < first) {
        second = first;
        first = dt;
      } else if (dt < second) {
        second = dt;
      }
    }
    if (static_cast<double>(second) > static_cast<double>(first) * (1.0 + config_.interval_jitter)) {
      return std::nullopt;
    }
    return first;
  }

  /**
   * @brief Choose a camera's frame for a target, or nothing if it is not yet certain.
   * @param forced The wait has run out: use the newest frame before the target
   * @param pick Set to the chosen frame
   * @return true if the camera is decided (pick set, or no frame at all when forced: pick stays null)
   */
  bool decide(const Camera & cam, int64_t target_ns, bool forced, const Frame *& pick) const
  {
    const Frame * prev = nullptr;
    const Frame * next = nullptr;
    const Frame * before_prev = nullptr;
    for (size_t i = 0; i < cam.frames.size(); ++i) {
      const auto & f = cam.frames[i];
      if (f.stamp_ns <= target_ns) {
        before_prev = prev;
        prev = &f;
      } else {
        next = &f;
        break;
      }
    }
    if (next) {
      pick = (prev && target_ns - prev->stamp_ns <= next->stamp_ns - target_ns) ? prev : next;
      return true;
    }
    const auto min_interval = prev ? confirmedMinInterval(cam) : std::nullopt;
    if (min_interval) {
      // The next frame is stamped at least one (jitter-reduced) minimum interval after prev, and one more
      // interval after the frame before it than there are between that frame and prev; the second bound keeps
      // this exact when prev itself was stamped late. Prev is no farther from the target than the next frame
      // when it is within half of the time to the earlier bound.
      const double step = (1.0 - config_.interval_jitter) * static_cast<double>(*min_interval);
      const double prev_ns = static_cast<double>(prev->stamp_ns);
      double earliest_next = prev_ns + step;
      if (before_prev) {
        const double between = static_cast<double>(prev->stamp_ns - before_prev->stamp_ns);
        const double intervals = std::max(1.0, std::round(between / static_cast<double>(*min_interval)));
        earliest_next = std::min(earliest_next, static_cast<double>(before_prev->stamp_ns) + (intervals + 1.0) * step);
      }
      if (
        static_cast<double>(target_ns - prev->stamp_ns) <=
        config_.early_fraction * std::max(0.0, earliest_next - prev_ns))
      {
        pick = prev;
        return true;
      }
    }
    if (!forced) {
      return false;
    }
    pick = prev;  // may be null: no frame at or before the target
    return true;
  }

  /**
   * @brief Decide the cameras still open for a waiting scan and, once all are decided, emit it.
   * @param forced The wait has run out; never returns kWaiting
   * @return Whether the scan is still waiting, was emitted to out, or must be dropped (and why)
   */
  Outcome complete(Pending & p, int64_t now_ns, bool forced, std::vector<MatchedSet> & out)
  {
    bool waiting = false;
    for (size_t i = 0; i < cameras_.size(); ++i) {
      if (p.chosen[i]) {
        continue;
      }
      const Frame * pick = nullptr;
      if (!decide(cameras_[i], p.targets_ns[i], forced, pick)) {
        waiting = true;
        continue;
      }
      if (!pick) {
        return Outcome::kNoFrame;
      }
      if (std::llabs(pick->stamp_ns - p.targets_ns[i]) > config_.tolerance_ns) {
        return Outcome::kOutOfTolerance;
      }
      p.chosen[i] = *pick;
    }
    if (waiting) {
      return Outcome::kWaiting;
    }
    MatchedSet set;
    set.scan_stamp_ns = p.stamp_ns;
    set.scan_arrival_ns = p.arrival_ns;
    set.decided_ns = now_ns;
    set.frames.reserve(cameras_.size());
    for (size_t i = 0; i < cameras_.size(); ++i) {
      set.max_gap_ns =
        std::max(set.max_gap_ns, static_cast<int64_t>(std::llabs(p.chosen[i]->stamp_ns - p.targets_ns[i])));
      set.frames.push_back(std::move(*p.chosen[i]));
    }
    set.scan = std::move(p.scan);
    set.targets_ns = std::move(p.targets_ns);
    out.push_back(std::move(set));
    ++counters_.matched;
    return Outcome::kEmitted;
  }

  void evaluate(int64_t now_ns, std::vector<MatchedSet> & out)
  {
    for (auto it = pending_.begin(); it != pending_.end();) {
      switch (complete(*it, now_ns, now_ns >= deadline(*it), out)) {
        case Outcome::kWaiting:
          ++it;
          continue;
        case Outcome::kNoFrame:
          ++counters_.dropped_no_frame;
          break;
        case Outcome::kOutOfTolerance:
          ++counters_.dropped_out_of_tolerance;
          break;
        case Outcome::kEmitted:
          break;
      }
      it = pending_.erase(it);
    }
  }

  CameraMatcherConfig config_;
  std::vector<Camera> cameras_;
  std::deque<Pending> pending_;
  std::optional<int64_t> last_scan_stamp_ns_;
  CameraMatcherCounters counters_;
};

}  // namespace wato::perception::bevfusion
