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

#ifndef BEHAVIOUR__UTILS__TYPES_HPP_
#define BEHAVIOUR__UTILS__TYPES_HPP_

#include <cstdint>
#include <memory>

#include <behaviortree_cpp/basic_types.h>

namespace behaviour::types
{

  // Enumeration for different entity types in the world model
  enum class EntityType : uint8_t
  {
    UNKNOWN = 0,
    CAR = 1,
    TRAFFIC_LIGHT = 2,
    // add other entity types as needed
  };

  // Enumeration for different lane transition types
  enum class LaneTransition : uint8_t
  {
    SUCCESSOR = 0, // follow route
    LEFT = 1,      // left lane change
    RIGHT = 2,     // right lane change
  };

  inline const char *toString(LaneTransition value)
  {
    switch (value)
    {
    case LaneTransition::SUCCESSOR:
      return "SUCCESSOR";
    case LaneTransition::LEFT:
      return "LEFT";
    case LaneTransition::RIGHT:
      return "RIGHT";
    default:
      return "UNKNOWN";
    }
  }

  // Enumeration for different traffic CONTROL element types
  enum class TrafficControlElementType : uint8_t
  {
    TRAFFIC_LIGHT = 0,
    STOP_SIGN = 1,
    YIELD = 2,
  };

  inline const char *toString(TrafficControlElementType value)
  {
    switch (value)
    {
    case TrafficControlElementType::TRAFFIC_LIGHT:
      return "TRAFFIC_LIGHT";
    case TrafficControlElementType::STOP_SIGN:
      return "STOP_SIGN";
    case TrafficControlElementType::YIELD:
      return "YIELD";
    default:
      return "UNKNOWN";
    }
  }

}

namespace BT
{
  template <>
  inline behaviour::types::LaneTransition convertFromString(StringView str)
  {
    if (str == "SUCCESSOR")
      return behaviour::types::LaneTransition::SUCCESSOR;
    if (str == "LEFT")
      return behaviour::types::LaneTransition::LEFT;
    if (str == "RIGHT")
      return behaviour::types::LaneTransition::RIGHT;
    throw BT::RuntimeError("Can't convert string [", str, "] to LaneTransition");
  }

  template <>
  inline behaviour::types::TrafficControlElementType convertFromString(StringView str)
  {
    if (str == "TRAFFIC_LIGHT")
      return behaviour::types::TrafficControlElementType::TRAFFIC_LIGHT;
    if (str == "STOP_SIGN")
      return behaviour::types::TrafficControlElementType::STOP_SIGN;
    if (str == "YIELD")
      return behaviour::types::TrafficControlElementType::YIELD;
    throw BT::RuntimeError("Can't convert string [", str, "] to TrafficControlElementType");
  }
} // namespace BT

#endif // BEHAVIOUR__UTILS__TYPES_HPP_
