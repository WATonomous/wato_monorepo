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

#ifndef WORLD_MODEL__PIPELINE__ENTITY_PERMANENCE_HPP_
#define WORLD_MODEL__PIPELINE__ENTITY_PERMANENCE_HPP_

#include <unordered_map>

#include "rclcpp/time.hpp"

namespace world_model
{

/**
 * @brief Manages entity lifetimes.
 *
 * Two responsibilities:
 *  - Trim detection history that exceeds the configured duration.
 *  - Prune entities that are empty or stale (no update within timeout).
 */
class EntityPermanence
{
public:
  EntityPermanence(double history_duration_sec, double timeout_sec)
  : history_duration_(history_duration_sec)
  , timeout_sec_(timeout_sec)
  {}

  /// @brief Returns the configured entity timeout in seconds.
  double timeoutSec() const { return timeout_sec_; }

  /**
   * @brief Trim old history entries and prune stale entities.
   */
  template <typename EntityT>
  void apply(std::unordered_map<int64_t, EntityT> & map, const rclcpp::Time & now)
  {
    for (auto it = map.begin(); it != map.end();) {
      if (isStale(it->second, now)) {
        it = map.erase(it);
      } else {
        trimHistory(it->second);
        ++it;
      }
    }
  }

private:
  /**
   * @brief Remove history entries that exceed the configured duration.
   *
   * Pops from the back (oldest) while the time span between newest and
   * oldest entries exceeds history_duration_, keeping at least one entry.
   *
   * @tparam EntityT Entity type with history deque.
   * @param entity Entity whose history to trim.
   */
  template <typename EntityT>
  void trimHistory(EntityT & entity) const
  {
    while (entity.history.size() > 1) {
      auto oldest = rclcpp::Time(entity.history.back().header.stamp);
      auto newest = rclcpp::Time(entity.history.front().header.stamp);
      if ((newest - oldest).seconds() > history_duration_) {
        entity.history.pop_back();
      } else {
        break;
      }
    }
  }

  /**
   * @brief Check if an entity is stale (empty or not updated within timeout).
   *
   * @tparam EntityT Entity type with empty() and timestamp() methods.
   * @param entity Entity to check.
   * @param now Current time for staleness comparison.
   * @return true if the entity should be pruned.
   */
  template <typename EntityT>
  bool isStale(const EntityT & entity, const rclcpp::Time & now) const
  {
    return entity.empty() || (now - entity.timestamp()).seconds() > timeout_sec_;
  }

  double history_duration_;
  double timeout_sec_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__PIPELINE__ENTITY_PERMANENCE_HPP_
