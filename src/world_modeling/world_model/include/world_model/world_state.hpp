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

#ifndef WORLD_MODEL__WORLD_STATE_HPP_
#define WORLD_MODEL__WORLD_STATE_HPP_

#include "world_model/types/types.hpp"

namespace world_model
{

/**
 * @brief Central thread-safe storage for world state data.
 *
 * Pure data store with concurrency management. Provides generic
 * buffer access for all entity types. Business logic belongs in
 * the interfaces (subscribers, publishers, services, workers).
 */
class WorldState
{
public:
  WorldState() = default;
  ~WorldState() = default;

  // Non-copyable, non-movable
  WorldState(const WorldState &) = delete;
  WorldState & operator=(const WorldState &) = delete;
  WorldState(WorldState &&) = delete;
  WorldState & operator=(WorldState &&) = delete;

  // ═══════════════════════════════════════════════════════════════════════════
  // GENERIC BUFFER ACCESS
  // ═══════════════════════════════════════════════════════════════════════════

  template <typename T>
  EntityBuffer<T> & buffer();
  template <typename T>
  const EntityBuffer<T> & buffer() const;

private:
  EntityBuffer<Car> cars_;
  EntityBuffer<Human> humans_;
  EntityBuffer<Bicycle> bicycles_;
  EntityBuffer<Motorcycle> motorcycles_;
  EntityBuffer<TrafficLight> traffic_lights_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// Template specializations for buffer access
// ═══════════════════════════════════════════════════════════════════════════════

template <>
inline EntityBuffer<Car> & WorldState::buffer<Car>()
{
  return cars_;
}

template <>
inline EntityBuffer<Human> & WorldState::buffer<Human>()
{
  return humans_;
}

template <>
inline EntityBuffer<Bicycle> & WorldState::buffer<Bicycle>()
{
  return bicycles_;
}

template <>
inline EntityBuffer<Motorcycle> & WorldState::buffer<Motorcycle>()
{
  return motorcycles_;
}

template <>
inline EntityBuffer<TrafficLight> & WorldState::buffer<TrafficLight>()
{
  return traffic_lights_;
}

template <>
inline const EntityBuffer<Car> & WorldState::buffer<Car>() const
{
  return cars_;
}

template <>
inline const EntityBuffer<Human> & WorldState::buffer<Human>() const
{
  return humans_;
}

template <>
inline const EntityBuffer<Bicycle> & WorldState::buffer<Bicycle>() const
{
  return bicycles_;
}

template <>
inline const EntityBuffer<Motorcycle> & WorldState::buffer<Motorcycle>() const
{
  return motorcycles_;
}

template <>
inline const EntityBuffer<TrafficLight> & WorldState::buffer<TrafficLight>() const
{
  return traffic_lights_;
}

}  // namespace world_model

#endif  // WORLD_MODEL__WORLD_STATE_HPP_
