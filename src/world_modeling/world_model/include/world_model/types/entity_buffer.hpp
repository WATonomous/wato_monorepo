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

#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace world_model
{

/**
 * @brief Thread-safe container for entities of a single type.
 *
 * Uses a shared_mutex for reader-writer locking:
 * - Multiple readers can access concurrently
 * - Writers get exclusive access
 *
 * @tparam T Entity type (must have id() method returning int64_t)
 */
template<typename T>
class EntityBuffer
{
public:
  void update(const T & entity)
  {
    std::unique_lock lock(mutex_);
    entities_[entity.id()] = entity;
  }

  void remove(int64_t id)
  {
    std::unique_lock lock(mutex_);
    entities_.erase(id);
  }

  std::optional<T> get(int64_t id) const
  {
    std::shared_lock lock(mutex_);
    auto it = entities_.find(id);
    if (it != entities_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<T> getAll() const
  {
    std::shared_lock lock(mutex_);
    std::vector<T> result;
    result.reserve(entities_.size());
    for (const auto & [id, entity] : entities_) {
      result.push_back(entity);
    }
    return result;
  }

  std::vector<T> getByLanelet(int64_t lanelet_id) const
  {
    std::shared_lock lock(mutex_);
    std::vector<T> result;
    for (const auto & [id, entity] : entities_) {
      if (entity.lanelet_id.has_value() && *entity.lanelet_id == lanelet_id) {
        result.push_back(entity);
      }
    }
    return result;
  }

  template<typename Predicate>
  void prune(Predicate should_remove)
  {
    std::unique_lock lock(mutex_);
    for (auto it = entities_.begin(); it != entities_.end(); ) {
      if (should_remove(it->second)) {
        it = entities_.erase(it);
      } else {
        ++it;
      }
    }
  }

  template<typename Func>
  void forEach(Func func)
  {
    std::unique_lock lock(mutex_);
    for (auto & [id, entity] : entities_) {
      func(entity);
    }
  }

  /**
   * @brief Atomically update an existing entity.
   *
   * @param id Entity ID to update
   * @param modifier Function that modifies the entity in-place
   * @return true if entity existed and was modified, false otherwise
   */
  template<typename Func>
  bool modify(int64_t id, Func modifier)
  {
    std::unique_lock lock(mutex_);
    auto it = entities_.find(id);
    if (it != entities_.end()) {
      modifier(it->second);
      return true;
    }
    return false;
  }

  /**
   * @brief Atomically insert or update an entity.
   *
   * If entity with ID exists, applies modifier to it.
   * If not, inserts default_entity and applies modifier to it.
   *
   * @param id Entity ID
   * @param default_entity Entity to insert if ID doesn't exist
   * @param modifier Function that modifies the entity in-place
   */
  template<typename Func>
  void upsert(int64_t id, T default_entity, Func modifier)
  {
    std::unique_lock lock(mutex_);
    auto [it, inserted] = entities_.try_emplace(id, std::move(default_entity));
    modifier(it->second);
  }

  template<typename Func>
  void forEachConst(Func func) const
  {
    std::shared_lock lock(mutex_);
    for (const auto & [id, entity] : entities_) {
      func(entity);
    }
  }

  size_t size() const
  {
    std::shared_lock lock(mutex_);
    return entities_.size();
  }

  bool empty() const
  {
    std::shared_lock lock(mutex_);
    return entities_.empty();
  }

  void clear()
  {
    std::unique_lock lock(mutex_);
    entities_.clear();
  }

private:
  mutable std::shared_mutex mutex_;
  std::unordered_map<int64_t, T> entities_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_BUFFER_HPP_
