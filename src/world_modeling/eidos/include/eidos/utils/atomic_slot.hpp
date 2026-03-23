#pragma once

#include <memory>
#include <mutex>

namespace eidos {

/**
 * @brief Single-writer, multi-reader slot for large payloads.
 *
 * Writer swaps in a new immutable shared_ptr. Readers get the latest snapshot.
 * Uses a spinlock-weight mutex (the critical section is just a pointer swap).
 *
 * Use for: gtsam::Values, or any type too large for a seqlock copy.
 */
template <typename T>
class AtomicSlot {
public:
  void store(T&& value) {
    auto ptr = std::make_shared<const T>(std::move(value));
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = std::move(ptr);
  }

  void store(const T& value) {
    auto ptr = std::make_shared<const T>(value);
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = std::move(ptr);
  }

  std::shared_ptr<const T> load() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.reset();
  }

private:
  mutable std::mutex mtx_;
  std::shared_ptr<const T> data_;
};

}  // namespace eidos
