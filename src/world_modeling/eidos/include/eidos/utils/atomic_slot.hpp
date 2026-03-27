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

#include <memory>
#include <mutex>
#include <utility>

namespace eidos
{

/**
 * @brief Single-writer, multi-reader slot for large payloads.
 *
 * Writer swaps in a new immutable shared_ptr. Readers get the latest snapshot.
 * Uses a spinlock-weight mutex (the critical section is just a pointer swap).
 *
 * Use for: gtsam::Values, or any type too large for a seqlock copy.
 */
template <typename T>
class AtomicSlot
{
public:
  void store(T && value)
  {
    auto ptr = std::make_shared<const T>(std::move(value));
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = std::move(ptr);
  }

  void store(const T & value)
  {
    auto ptr = std::make_shared<const T>(value);
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = std::move(ptr);
  }

  std::shared_ptr<const T> load() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.reset();
  }

private:
  mutable std::mutex mtx_;
  std::shared_ptr<const T> data_;
};

}  // namespace eidos
