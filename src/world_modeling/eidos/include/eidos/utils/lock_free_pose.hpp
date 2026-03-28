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

#include <array>
#include <atomic>
#include <optional>

namespace eidos
{

/**
 * @brief Single-writer, multi-reader lock-free storage for gtsam::Pose3.
 *
 * Uses a 3-slot ring buffer: the writer writes to the next available slot
 * and atomically updates the read index. Readers always read a complete,
 * consistent pose from the current read slot — no retries, no torn reads.
 *
 * Constraints:
 * - Exactly one thread may call store() (single writer).
 * - Any number of threads may call load() concurrently (multi reader).
 */
class LockFreePose
{
public:
  /// @brief Store a pose (single-writer only).
  /// @param p The gtsam::Pose3 to store.
  void store(const gtsam::Pose3 & p)
  {
    // Write to the next slot (not the one readers are currently using)
    int next = (write_idx_ + 1) % kNumSlots;
    slots_[next] = p;
    // Atomically publish the new slot for readers
    read_idx_.store(next, std::memory_order_release);
    write_idx_ = next;
    has_value_.store(true, std::memory_order_release);
  }

  /// @brief Load the latest pose snapshot.
  /// @return The stored pose, or std::nullopt if no pose has been stored yet.
  std::optional<gtsam::Pose3> load() const
  {
    if (!has_value_.load(std::memory_order_acquire)) return std::nullopt;
    int idx = read_idx_.load(std::memory_order_acquire);
    return slots_[idx];
  }

private:
  static constexpr int kNumSlots = 3;
  std::array<gtsam::Pose3, kNumSlots> slots_;
  std::atomic<int> read_idx_{0};
  int write_idx_{0};  ///< Only accessed by the single writer thread
  std::atomic<bool> has_value_{false};
};

}  // namespace eidos
