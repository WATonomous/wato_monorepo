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

#ifndef BEHAVIOUR__UTILS__AREA_OCCUPANCY_UTILS_HPP_
#define BEHAVIOUR__UTILS__AREA_OCCUPANCY_UTILS_HPP_

#include <string>
#include <vector>

#include "world_model_msgs/msg/area_occupancy_info.hpp"

namespace behaviour::area_occupancy_utils
{
inline const world_model_msgs::msg::AreaOccupancyInfo * getAreaByName(
  const std::vector<world_model_msgs::msg::AreaOccupancyInfo> & areas, const std::string & area_name)
{
  for (const auto & area : areas) {
    if (area.name == area_name) {
      return &area;
    }
  }
  return nullptr;
}

inline bool isAreaOccupied(
  const std::vector<world_model_msgs::msg::AreaOccupancyInfo> & areas, const std::string & area_name)
{
  const auto * area = getAreaByName(areas, area_name);
  return area != nullptr && area->is_occupied;
}
}  // namespace behaviour::area_occupancy_utils

#endif  // BEHAVIOUR__UTILS__AREA_OCCUPANCY_UTILS_HPP_
