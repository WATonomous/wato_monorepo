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

#ifndef BEHAVIOUR__UTILS__LANELET_UTILS_HPP_
#define BEHAVIOUR__UTILS__LANELET_UTILS_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "behaviour/utils/types.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour::utils::lanelet
{
enum class RightOfWayRole : uint8_t
{
  NONE = 0,
  YIELD = 1,
  RIGHT_OF_WAY = 2,
};

inline std::optional<std::string> getAttributeValue(
  const lanelet_msgs::msg::RegulatoryElement & reg_elem, const std::string & key)
{
  const auto attribute_count = std::min(reg_elem.attribute_keys.size(), reg_elem.attribute_values.size());
  for (std::size_t i = 0; i < attribute_count; ++i) {
    if (reg_elem.attribute_keys[i] == key) {
      return reg_elem.attribute_values[i];
    }
  }
  return std::nullopt;
}

inline bool containsLaneletId(const std::vector<int64_t> & lanelet_ids, int64_t lanelet_id)
{
  return std::find(lanelet_ids.begin(), lanelet_ids.end(), lanelet_id) != lanelet_ids.end();
}

inline RightOfWayRole getRightOfWayRole(
  const lanelet_msgs::msg::RegulatoryElement & reg_elem, int64_t lanelet_id)
{
  if (reg_elem.subtype != "right_of_way") {
    return RightOfWayRole::NONE;
  }

  if (containsLaneletId(reg_elem.yield_lanelet_ids, lanelet_id)) {
    return RightOfWayRole::YIELD;
  }

  if (containsLaneletId(reg_elem.right_of_way_lanelet_ids, lanelet_id)) {
    return RightOfWayRole::RIGHT_OF_WAY;
  }

  return RightOfWayRole::NONE;
}

inline std::optional<types::TrafficControlElementType> getTrafficControlElementType(
  const lanelet_msgs::msg::RegulatoryElement & reg_elem)
{
  if (reg_elem.subtype == "traffic_light") {
    return types::TrafficControlElementType::TRAFFIC_LIGHT;
  }

  if (reg_elem.subtype == "stop_sign" || reg_elem.subtype == "all_way_stop") {
    return types::TrafficControlElementType::STOP_SIGN;
  }

  if (reg_elem.subtype == "yield") {
    return types::TrafficControlElementType::YIELD;
  }

  if (reg_elem.subtype == "traffic_sign") {
    const auto sign_type = getAttributeValue(reg_elem, "sign_type");
    if (sign_type && *sign_type == "stop") {
      return types::TrafficControlElementType::STOP_SIGN;
    }
    if (sign_type && *sign_type == "yield") {
      return types::TrafficControlElementType::YIELD;
    }
    return std::nullopt;
  }

  if (reg_elem.subtype == "yield" || reg_elem.subtype == "right_of_way") {
    return types::TrafficControlElementType::YIELD;
  }

  return std::nullopt;
}
}  // namespace behaviour::utils::lanelet

#endif  // BEHAVIOUR__UTILS__LANELET_UTILS_HPP_
