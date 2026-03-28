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
 * @brief Single-writer, multi-reader lock-free storage for gtsam::Pose3.
 *
 * Uses a seqlock pattern: the writer increments a sequence counter to odd
 * before writing and back to even after. A reader that observes an odd
 * counter (writer active) or a counter change (write happened during read)
 * yields and retries. The copy takes nanoseconds so retries are rare.
 *
 * Constraints:
 * - Exactly one thread may call store() (single writer).
 * - Any number of threads may call load() concurrently (multi reader).
 */
class LockFreePose
{
public:
  /**
   * @brief Store a pose into the slot (single-writer only).
   * @param p The gtsam::Pose3 to store.
   */
  void store(const gtsam::Pose3 & p)
  {
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    pose_ = p;
    has_value_.store(true, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);  // even = done
  }

  /**
   * @brief Load the latest pose snapshot.
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
  std::atomic<uint64_t> seq_{0};
  gtsam::Pose3 pose_;
  std::atomic<bool> has_value_{false};
};

}  // namespace eidos
