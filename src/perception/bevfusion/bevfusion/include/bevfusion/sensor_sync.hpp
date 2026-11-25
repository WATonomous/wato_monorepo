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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wato::perception::bevfusion
{

// Timestamp metadata only: payloads are owning handles, never pixel/point arrays.
// The caller serializes access. Sensor stamps use a common clock; expiry uses steady time.
template <typename ImageHandle, typename LidarHandle>
class SensorSync
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Config
  {
    size_t camera_count{0};
    size_t camera_capacity{10};
    size_t lidar_capacity{10};
    int64_t camera_tolerance_ns{50000000};
    int64_t lidar_tolerance_ns{200000000};
    int64_t max_wait_ns{200000000};
    int64_t max_age_ns{1000000000};
  };

  struct Stats
  {
    uint64_t matched{0};
    uint64_t image_overflow{0};
    uint64_t image_expired{0};
    uint64_t lidar_overflow{0};
    uint64_t lidar_expired{0};
    uint64_t lidar_duplicates{0};
    uint64_t clock_resets{0};
    int64_t camera_span_ns{0};
    int64_t lidar_offset_ns{0};
    int64_t wait_ns{0};
  };

  struct Match
  {
    std::vector<ImageHandle> images;
    LidarHandle lidar;
    TimePoint arrival;
    uint64_t generation;
  };

  explicit SensorSync(const Config & config)
  : config_(config)
  , images_(config.camera_count)
  , last_camera_stamp_(config.camera_count)
  {
    if (
      !config.camera_count || !config.camera_capacity || !config.lidar_capacity || config.camera_tolerance_ns < 0 ||
      config.lidar_tolerance_ns < 0 || config.max_wait_ns <= 0 || config.max_age_ns < config.max_wait_ns)
    {
      throw std::invalid_argument("Invalid sensor synchronization limits");
    }
  }

  void addImage(size_t camera, int64_t stamp, ImageHandle image, TimePoint now = Clock::now())
  {
    if (camera >= images_.size() || stamp < 0) throw std::invalid_argument("Invalid camera index or timestamp");
    checkClock(stamp, last_camera_stamp_[camera]);
    last_camera_stamp_[camera] = std::max(stamp, last_camera_stamp_[camera].value_or(stamp));
    expire(now);
    insert(images_[camera], Entry<ImageHandle>{stamp, std::move(image), now});
    if (images_[camera].size() > config_.camera_capacity) {
      images_[camera].pop_front();
      ++stats_.image_overflow;
    }
  }

  void addLidar(int64_t stamp, LidarHandle lidar, TimePoint now = Clock::now())
  {
    if (stamp < 0) throw std::invalid_argument("Invalid lidar timestamp");
    checkClock(stamp, last_lidar_stamp_);
    last_lidar_stamp_ = std::max(stamp, last_lidar_stamp_.value_or(stamp));
    expire(now);
    if (std::any_of(seen_lidar_.begin(), seen_lidar_.end(), [stamp](const auto & e) { return e.first == stamp; })) {
      ++stats_.lidar_duplicates;
      return;
    }
    seen_lidar_.emplace_back(stamp, now);
    // Bound duplicate tracking even for extreme input rates.
    if (seen_lidar_.size() > 4 * config_.lidar_capacity) seen_lidar_.pop_front();
    insert(lidar_, Entry<LidarHandle>{stamp, std::move(lidar), now});
    if (lidar_.size() > config_.lidar_capacity) {
      lidar_.pop_front();
      ++stats_.lidar_overflow;
    }
  }

  std::optional<Match> takeMatch(TimePoint now = Clock::now())
  {
    expire(now);
    if (lidar_.empty()) return std::nullopt;
    std::vector<int64_t> starts;
    for (const auto & queue : images_) {
      if (queue.empty()) return std::nullopt;
      for (const auto & image : queue) starts.push_back(image.stamp);
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());

    // Enumerate bounded camera windows, not the exponential product of all histories.
    // In each window choose each camera's nearest image to the candidate lidar scan.
    std::optional<Match> best;
    int64_t best_distance = std::numeric_limits<int64_t>::max();
    int64_t best_span = std::numeric_limits<int64_t>::max();
    int64_t best_offset = 0;
    size_t best_lidar = 0;
    for (size_t scan = 0; scan < lidar_.size(); ++scan) {
      const auto & cloud = lidar_[scan];
      for (const int64_t start : starts) {
        std::vector<ImageHandle> selected;
        std::vector<int64_t> stamps;
        for (const auto & queue : images_) {
          const Entry<ImageHandle> * nearest = nullptr;
          int64_t distance = std::numeric_limits<int64_t>::max();
          for (const auto & image : queue) {
            if (image.stamp < start || image.stamp - start > config_.camera_tolerance_ns) continue;
            const int64_t delta = difference(image.stamp, cloud.stamp);
            if (delta <= config_.lidar_tolerance_ns && delta < distance) {
              nearest = &image;
              distance = delta;
            }
          }
          if (!nearest) break;
          selected.push_back(nearest->payload);
          stamps.push_back(nearest->stamp);
        }
        if (selected.size() != images_.size()) continue;
        std::sort(stamps.begin(), stamps.end());
        const int64_t median =
          stamps[(stamps.size() - 1) / 2] + (stamps[stamps.size() / 2] - stamps[(stamps.size() - 1) / 2]) / 2;
        const int64_t distance = difference(median, cloud.stamp);
        const int64_t span = stamps.back() - stamps.front();
        if (distance < best_distance || (distance == best_distance && span < best_span)) {
          best = Match{std::move(selected), cloud.payload, cloud.arrival, generation_};
          best_distance = distance;
          best_span = span;
          best_offset = std::max(difference(stamps.front(), cloud.stamp), difference(stamps.back(), cloud.stamp));
          best_lidar = scan;
        }
      }
    }
    if (best) {
      lidar_.erase(lidar_.begin() + best_lidar);
      ++stats_.matched;
      stats_.camera_span_ns = best_span;
      stats_.lidar_offset_ns = best_offset;
      stats_.wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - best->arrival).count();
    }
    // Images may be reused by consecutive lidar scans; only scans are consumed.
    return best;
  }

  void expire(TimePoint now = Clock::now())
  {
    for (auto & queue : images_) {
      const auto previous = queue.size();
      queue.erase(
        std::remove_if(
          queue.begin(),
          queue.end(),
          [&](const auto & e) { return now - e.arrival >= std::chrono::nanoseconds(config_.max_age_ns); }),
        queue.end());
      stats_.image_expired += previous - queue.size();
    }
    const auto previous = lidar_.size();
    lidar_.erase(
      std::remove_if(
        lidar_.begin(),
        lidar_.end(),
        [&](const auto & e) { return now - e.arrival >= std::chrono::nanoseconds(config_.max_wait_ns); }),
      lidar_.end());
    stats_.lidar_expired += previous - lidar_.size();
    while (!seen_lidar_.empty() && now - seen_lidar_.front().second >= std::chrono::nanoseconds(config_.max_age_ns)) {
      seen_lidar_.pop_front();
    }
  }

  bool camerasActive(TimePoint now = Clock::now()) const
  {
    return std::all_of(images_.begin(), images_.end(), [&](const auto & queue) {
      return std::any_of(queue.begin(), queue.end(), [&](const auto & e) {
        return now - e.arrival < std::chrono::nanoseconds(config_.max_age_ns);
      });
    });
  }

  // The batched camera transport resolved this scan; it is no longer awaiting GPU images.
  void discardLidar(int64_t stamp)
  {
    lidar_.erase(
      std::remove_if(lidar_.begin(), lidar_.end(), [stamp](const auto & entry) { return entry.stamp == stamp; }),
      lidar_.end());
  }

  void clear()
  {
    for (auto & queue : images_) queue.clear();
    lidar_.clear();
    seen_lidar_.clear();
    std::fill(last_camera_stamp_.begin(), last_camera_stamp_.end(), std::nullopt);
    last_lidar_stamp_.reset();
    ++generation_;
  }

  const Stats & stats() const
  {
    return stats_;
  }

  uint64_t generation() const
  {
    return generation_;
  }

  size_t pendingLidar() const
  {
    return lidar_.size();
  }

  size_t cameraSize(size_t camera) const
  {
    return images_.at(camera).size();
  }

private:
  template <typename Handle>
  struct Entry
  {
    int64_t stamp;
    Handle payload;
    TimePoint arrival;
  };

  static int64_t difference(int64_t a, int64_t b)
  {
    return a >= b ? a - b : b - a;
  }

  template <typename Handle>
  static void insert(std::deque<Entry<Handle>> & queue, Entry<Handle> entry)
  {
    const auto pos = std::lower_bound(
      queue.begin(), queue.end(), entry.stamp, [](const auto & e, int64_t stamp) { return e.stamp < stamp; });
    if (pos == queue.end() || pos->stamp != entry.stamp) queue.insert(pos, std::move(entry));
  }

  void checkClock(int64_t stamp, const std::optional<int64_t> & previous)
  {
    // Allow packet reordering; a large rewind starts a new bag/sensor epoch.
    if (previous && *previous > stamp && *previous - stamp > std::max<int64_t>(2000000000, config_.max_age_ns)) {
      clear();
      ++stats_.clock_resets;
    }
  }

  Config config_;
  std::vector<std::deque<Entry<ImageHandle>>> images_;
  std::deque<Entry<LidarHandle>> lidar_;
  std::deque<std::pair<int64_t, TimePoint>> seen_lidar_;
  std::vector<std::optional<int64_t>> last_camera_stamp_;
  std::optional<int64_t> last_lidar_stamp_;
  uint64_t generation_{0};
  Stats stats_;
};

}  // namespace wato::perception::bevfusion
