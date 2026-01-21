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

#ifndef WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_
#define WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_

#include "world_model/lanelet_handler.hpp"
#include "world_model/world_state.hpp"

namespace world_model
{

// ═══════════════════════════════════════════════════════════════════════════════
// ACCESS WRAPPERS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Read-only accessor for WorldState.
 *
 * Provides const-only access to entity buffers. The underlying EntityBuffer
 * handles reader locking internally (shared_lock).
 */
class WorldStateReader
{
public:
  explicit WorldStateReader(const WorldState * state) : state_(state) {}

  template<typename T>
  const EntityBuffer<T> & buffer() const { return state_->buffer<T>(); }

private:
  const WorldState * state_;
};

/**
 * @brief Read-write accessor for WorldState.
 *
 * Provides full access to entity buffers. The underlying EntityBuffer
 * handles writer locking internally (unique_lock).
 */
class WorldStateWriter
{
public:
  explicit WorldStateWriter(WorldState * state) : state_(state) {}

  template<typename T>
  EntityBuffer<T> & buffer() { return state_->buffer<T>(); }

  template<typename T>
  const EntityBuffer<T> & buffer() const { return state_->buffer<T>(); }

private:
  WorldState * state_;
};

/**
 * @brief Read-only accessor for LaneletHandler.
 *
 * LaneletHandler is immutable after map load, so only read access is provided.
 */
class LaneletReader
{
public:
  explicit LaneletReader(const LaneletHandler * handler) : handler_(handler) {}

  bool isMapLoaded() const { return handler_->isMapLoaded(); }

  std::optional<int64_t> findNearestLaneletId(
    const geometry_msgs::msg::Point & point) const
  {
    return handler_->findNearestLaneletId(point);
  }

  std::optional<lanelet::ConstLanelet> getLaneletById(int64_t id) const
  {
    return handler_->getLaneletById(id);
  }

  std::vector<lanelet::ConstLanelet> getLaneletsInRadius(
    const geometry_msgs::msg::Point & center, double radius_m) const
  {
    return handler_->getLaneletsInRadius(center, radius_m);
  }

  lanelet_msgs::msg::Lanelet toLaneletMsg(const lanelet::ConstLanelet & ll) const
  {
    return handler_->toLaneletMsg(ll);
  }

  // Service methods
  lanelet_msgs::srv::GetRoute::Response getRoute(int64_t from_id, int64_t to_id) const
  {
    return handler_->getRoute(from_id, to_id);
  }

  lanelet_msgs::srv::GetCorridor::Response getCorridor(
    int64_t from_id, int64_t to_id, double max_length_m,
    double sample_spacing_m) const
  {
    return handler_->getCorridor(from_id, to_id, max_length_m, sample_spacing_m);
  }

  lanelet_msgs::srv::GetLaneletsByRegElem::Response getLaneletsByRegElem(int64_t reg_elem_id) const
  {
    return handler_->getLaneletsByRegElem(reg_elem_id);
  }

private:
  const LaneletHandler * handler_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// BASE INTERFACE
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Base interface for all ROS interface components.
 *
 * Provides common lifecycle management for publishers, subscribers,
 * services, and workers.
 *
 * Derived classes should use the typed accessor wrappers (WorldStateReader,
 * WorldStateWriter, LaneletReader) to declare their access patterns at
 * construction time, enforcing concurrency safety at compile time.
 */
class InterfaceBase
{
public:
  virtual ~InterfaceBase() = default;

  /**
   * @brief Activate the interface (start publishing, subscribe, etc.)
   */
  virtual void activate() {}

  /**
   * @brief Deactivate the interface (stop timers, cancel subscriptions, etc.)
   */
  virtual void deactivate() {}
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__INTERFACE_BASE_HPP_
