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

#ifndef WORLD_MODEL__TYPES__ENTITY_HPP_
#define WORLD_MODEL__TYPES__ENTITY_HPP_

#include <optional>

#include "world_model/types/entity_type.hpp"

namespace world_model
{

/**
 * @brief Base class for all tracked entities.
 *
 * Entities represent objects in the world that are tracked over time.
 * They may be enriched with lanelet context by WorldState.
 */
class Entity
{
public:
  virtual ~Entity() = default;
  virtual EntityType type() const = 0;

  // Enriched by WorldState
  std::optional<int64_t> lanelet_id;
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_HPP_
