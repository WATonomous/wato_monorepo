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

#ifndef WORLD_MODEL__TYPES__ENTITY_TYPE_HPP_
#define WORLD_MODEL__TYPES__ENTITY_TYPE_HPP_

#include <cstdint>

namespace world_model
{

enum class EntityType : uint8_t {
  UNKNOWN = 0,
  CAR = 1,
  HUMAN = 2,
  BICYCLE = 3,
  MOTORCYCLE = 4,
  TRAFFIC_LIGHT = 5
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_TYPE_HPP_
