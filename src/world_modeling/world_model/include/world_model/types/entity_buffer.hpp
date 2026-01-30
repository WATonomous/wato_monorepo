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

#ifndef WORLD_MODEL__TYPES__ENTITY_BUFFER_HPP_
#define WORLD_MODEL__TYPES__ENTITY_BUFFER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace world_model
{

/**
 * @brief Thread-safe container for entities of a single type.
 *
 * Uses a copy-on-write pattern with an atomic shared pointer:
 * - Readers atomically load a snapshot and iterate without any lock
 * - Writers hold a mutex, copy the map, mutate the copy, then atomically swap
 *
 * @tparam T Entity type (must have id() method returning int64_t)
 */
template <typename T>
class EntityBuffer
{
  using Map = std::unordered_map<int64_t, T>;
  using MapPtr = std::shared_ptr<const Map>;

public:
  EntityBuffer()
  : data_(std::make_shared<const Map>())
  {
  }

  void update(const T & entity)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto copy = std::make_shared<Map>(*std::atomic_load(&data_));
    (*copy)[entity.id()] = entity;
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  void remove(int64_t id)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto snapshot = std::atomic_load(&data_);
    if (snapshot->find(id) == snapshot->end()) {
      return;
    }
    auto copy = std::make_shared<Map>(*snapshot);
    copy->erase(id);
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  std::optional<T> get(int64_t id) const
  {
    auto snapshot = std::atomic_load(&data_);
    auto it = snapshot->find(id);
    if (it != snapshot->end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<T> getAll() const
  {
    auto snapshot = std::atomic_load(&data_);
    std::vector<T> result;
    result.reserve(snapshot->size());
    for (const auto & [id, entity] : *snapshot) {
      result.push_back(entity);
    }
    return result;
  }

  std::vector<T> getByLanelet(int64_t lanelet_id) const
  {
    auto snapshot = std::atomic_load(&data_);
    std::vector<T> result;
    for (const auto & [id, entity] : *snapshot) {
      if (entity.lanelet_id.has_value() && *entity.lanelet_id == lanelet_id) {
        result.push_back(entity);
      }
    }
    return result;
  }

  template <typename Predicate>
  void prune(Predicate should_remove)
  {
    auto snapshot = std::atomic_load(&data_);

    // Fast path: check if anything needs pruning at all
    bool needs_pruning = false;
    for (const auto & [id, entity] : *snapshot) {
      if (should_remove(entity)) {
        needs_pruning = true;
        break;
      }
    }
    if (!needs_pruning) {
      return;
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    // Re-read under the lock in case another writer changed things
    snapshot = std::atomic_load(&data_);
    auto copy = std::make_shared<Map>();
    copy->reserve(snapshot->size());
    for (const auto & [id, entity] : *snapshot) {
      if (!should_remove(entity)) {
        copy->emplace(id, entity);
      }
    }
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  template <typename Func>
  void forEach(Func func)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto copy = std::make_shared<Map>(*std::atomic_load(&data_));
    for (auto & [id, entity] : *copy) {
      func(entity);
    }
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  template <typename Func>
  bool modify(int64_t id, Func modifier)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto snapshot = std::atomic_load(&data_);
    auto it = snapshot->find(id);
    if (it != snapshot->end()) {
      auto copy = std::make_shared<Map>(*snapshot);
      modifier((*copy)[id]);
      std::atomic_store(&data_, MapPtr(std::move(copy)));
      return true;
    }
    return false;
  }

  template <typename Func>
  void upsert(int64_t id, T default_entity, Func modifier)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto copy = std::make_shared<Map>(*std::atomic_load(&data_));
    auto [it, inserted] = copy->try_emplace(id, std::move(default_entity));
    modifier(it->second);
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  /**
   * @brief Apply multiple writes under a single copy.
   *
   * Copies the map once, passes a mutable reference to the caller,
   * then atomically publishes the result. Use this when applying
   * many mutations (e.g. an entire Detection3DArray) to avoid
   * O(n) copy per individual upsert.
   */
  template <typename Func>
  void batch(Func func)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    auto copy = std::make_shared<Map>(*std::atomic_load(&data_));
    func(*copy);
    std::atomic_store(&data_, MapPtr(std::move(copy)));
  }

  template <typename Func>
  void forEachConst(Func func) const
  {
    auto snapshot = std::atomic_load(&data_);
    for (const auto & [id, entity] : *snapshot) {
      func(entity);
    }
  }

  size_t size() const
  {
    auto snapshot = std::atomic_load(&data_);
    return snapshot->size();
  }

  bool empty() const
  {
    auto snapshot = std::atomic_load(&data_);
    return snapshot->empty();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    std::atomic_store(&data_, std::make_shared<const Map>());
  }

private:
  std::mutex write_mutex_;
  mutable MapPtr data_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_BUFFER_HPP_
