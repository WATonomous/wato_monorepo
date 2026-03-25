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

namespace eidos
{

/**
 * @brief Single-writer, multi-reader lock-free storage for gtsam::Pose3.
 *
 * Uses a seqlock pattern: the writer increments a sequence counter before and
 * after writing. A reader that catches the writer mid-copy simply retries.
 * Since gtsam::Pose3 is ~100 bytes, the copy takes nanoseconds — the reader
 * effectively never blocks.
 *
 * Constraints:
 * - Exactly one thread may call store() (single writer).
 * - Any number of threads may call load() concurrently (multi reader).
 */
class LockFreePose
{
public:
  void store(const gtsam::Pose3 & p)
  {
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    pose_ = p;
    has_value_.store(true, std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);  // even = done
  }

  std::optional<gtsam::Pose3> load() const
  {
    gtsam::Pose3 result;
    bool valid;
    uint64_t s;
    do {
      s = seq_.load(std::memory_order_acquire);
      if (s & 1) continue;  // writer active, retry
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
