#pragma once

#include <atomic>
#include <optional>

#include <gtsam/geometry/Pose3.h>

namespace eidos {

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
class LockFreePose {
public:
  void store(const gtsam::Pose3& p) {
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    pose_ = p;
    has_value_ = true;
    seq_.fetch_add(1, std::memory_order_release);  // even = done
  }

  std::optional<gtsam::Pose3> load() const {
    gtsam::Pose3 result;
    bool valid;
    uint64_t s;
    do {
      s = seq_.load(std::memory_order_acquire);
      if (s & 1) continue;  // writer active, retry
      result = pose_;
      valid = has_value_;
    } while (seq_.load(std::memory_order_acquire) != s);
    return valid ? std::optional(result) : std::nullopt;
  }

private:
  std::atomic<uint64_t> seq_{0};
  gtsam::Pose3 pose_;
  bool has_value_ = false;
};

}  // namespace eidos
