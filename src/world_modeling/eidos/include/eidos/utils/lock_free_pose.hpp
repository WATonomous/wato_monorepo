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

#include <atomic>
#include <optional>
#include <thread>

namespace eidos
{

/**
 * @brief Multi-writer, multi-reader lock-free storage for gtsam::Pose3.
 *
 * Writers serialize via an atomic spinlock, then use a seqlock to publish.
 * Readers are fully lock-free — they observe the sequence counter and retry
 * (with yield) if a write is in progress.
 *
 * Safe to call store() from any thread. Safe to call load() from any thread.
 */
class LockFreePose
{
public:
  /**
   * @brief Store a pose (safe from any thread).
   * @param p The gtsam::Pose3 to store.
   */
  void store(const gtsam::Pose3 & p)
  {
    // Serialize writers
    while (write_lock_.test_and_set(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    pose_ = p;
    has_value_.store(true, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);  // even = done
    write_lock_.clear(std::memory_order_release);
  }

  /**
   * @brief Load the latest pose snapshot (lock-free, any thread).
   * @return The stored pose, or std::nullopt if no pose has been stored yet.
   */
  std::optional<gtsam::Pose3> load() const
  {
    gtsam::Pose3 result;
    bool valid;
    uint64_t s;
    do {
      s = seq_.load(std::memory_order_acquire);
      if (s & 1) {
        std::this_thread::yield();
        continue;
      }
      result = pose_;
      valid = has_value_.load(std::memory_order_relaxed);
    } while (seq_.load(std::memory_order_acquire) != s);
    return valid ? std::optional(result) : std::nullopt;
  }

private:
  std::atomic_flag write_lock_ = ATOMIC_FLAG_INIT;
  std::atomic<uint64_t> seq_{0};
  gtsam::Pose3 pose_;
  std::atomic<bool> has_value_{false};
};

}  // namespace eidos
