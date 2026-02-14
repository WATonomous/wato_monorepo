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

#ifndef BEHAVIOUR__UTILS__OBJECT_UTILS_HPP_
#define BEHAVIOUR__UTILS__OBJECT_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour::world_objects
{
  inline bool isVehicle(const world_model_msgs::msg::WorldObject &object, int hypothesis_index)
  {
    if (hypothesis_index < 0)
    {
      return false;
    }

    const std::size_t idx = static_cast<std::size_t>(hypothesis_index);
    if (idx >= object.detection.results.size())
    {
      return false;
    }

    const auto &class_id = object.detection.results[idx].hypothesis.class_id;
    return class_id == "car" || class_id == "vehicle" || class_id == "truck";
  }

  inline std::vector<const world_model_msgs::msg::WorldObject *> getCarsByLanelet(
      const std::vector<world_model_msgs::msg::WorldObject> &objects, int hypothesis_index, int64_t lanelet_id)
  {
    std::vector<const world_model_msgs::msg::WorldObject *> cars;
    cars.reserve(objects.size());

    for (const auto &object : objects)
    {
      if (object.lanelet_ahead.current_lanelet_id != lanelet_id)
      {
        continue;
      }
      if (!isVehicle(object, hypothesis_index))
      {
        continue;
      }
      cars.push_back(&object);
    }

    return cars;
  }

  inline const world_model_msgs::msg::WorldObject *getTrafficLightById(
      const std::vector<world_model_msgs::msg::WorldObject> &objects, int64_t reg_elem_id)
  {
    const std::string reg_elem_id_str = std::to_string(reg_elem_id);
    for (const auto &object : objects)
    {
      if (object.detection.id == reg_elem_id_str)
      {
        return &object;
      }
    }
    return nullptr;
  }

  inline std::optional<std::string> getTrafficLightState(
      int64_t reg_elem_id,
      int state_hypothesis_index,
      const std::vector<world_model_msgs::msg::WorldObject> &objects)
  {
    if (state_hypothesis_index < 0)
    {
      return std::nullopt;
    }

    const auto *traffic_light = getTrafficLightById(objects, reg_elem_id);
    if (traffic_light == nullptr)
    {
      return std::nullopt;
    }

    const std::size_t idx = static_cast<std::size_t>(state_hypothesis_index);
    if (idx >= traffic_light->detection.results.size())
    {
      return std::nullopt;
    }

    return traffic_light->detection.results[idx].hypothesis.class_id;
  }
} // namespace behaviour::world_objects

#endif // BEHAVIOUR__UTILS__OBJECT_UTILS_HPP_
