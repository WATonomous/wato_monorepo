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

#ifndef WORLD_MODEL__PIPELINE__ENTITY_POPULATOR_HPP_
#define WORLD_MODEL__PIPELINE__ENTITY_POPULATOR_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/time.hpp"
#include "world_model/types/entity_3d.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace world_model
{

/**
 * @brief Upserts incoming WorldObjects into entity maps.
 *
 * For each object: parses the tracking ID, creates or updates the
 * entity, pushes the detection onto history, stores predictions,
 * and applies per-entity customization (e.g. traffic light state).
 */
class EntityPopulator
{
public:
  /**
   * @brief Upsert a batch of world objects into the entity map.
   *
   * For each object, parses the tracking ID, creates or updates the entity,
   * pushes the detection onto history, stores predictions, and applies
   * per-entity type customization (e.g. traffic light state parsing).
   *
   * @tparam EntityT Entity class (e.g. Car, Human, TrafficLight).
   * @param map Entity map to insert/update into.
   * @param objects Pointers to WorldObject messages to process.
   * @param msg_header Message header used as fallback for stamp/frame_id.
   */
  template <typename EntityT>
  void populate(
    std::unordered_map<int64_t, EntityT> & map,
    const std::vector<const world_model_msgs::msg::WorldObject *> & objects,
    const std_msgs::msg::Header & msg_header)
  {
    for (const auto * obj : objects) {
      try {
        EntityT & entity = upsertBase(map, *obj, msg_header);
        customize(entity, obj->detection);
      } catch (const std::exception &) {
        // Skip detections with non-numeric or empty IDs
        continue;
      }
    }
  }

private:
  /// @brief Per-entity customization hook (no-op for most types).
  static void customize(Unknown & /*entity*/, const vision_msgs::msg::Detection3D & /*det*/)
  {}

  /// @copydoc customize(Unknown&, const vision_msgs::msg::Detection3D&)
  static void customize(Car & /*entity*/, const vision_msgs::msg::Detection3D & /*det*/)
  {}

  /// @copydoc customize(Unknown&, const vision_msgs::msg::Detection3D&)
  static void customize(Human & /*entity*/, const vision_msgs::msg::Detection3D & /*det*/)
  {}

  /// @copydoc customize(Unknown&, const vision_msgs::msg::Detection3D&)
  static void customize(Bicycle & /*entity*/, const vision_msgs::msg::Detection3D & /*det*/)
  {}

  /// @copydoc customize(Unknown&, const vision_msgs::msg::Detection3D&)
  static void customize(Motorcycle & /*entity*/, const vision_msgs::msg::Detection3D & /*det*/)
  {}

  /**
   * @brief Customize a TrafficLight entity after upsert.
   *
   * Parses the detection class ID to set the traffic light state
   * (RED, YELLOW, GREEN, UNKNOWN) and confidence score.
   *
   * @param tl TrafficLight entity to customize.
   * @param det Detection with classification results.
   */
  static void customize(TrafficLight & tl, const vision_msgs::msg::Detection3D & det)
  {
    if (!det.results.empty()) {
      tl.confidence = det.results[0].hypothesis.score;

      const std::string & class_id = det.results[0].hypothesis.class_id;
      if (class_id == "red" || class_id == "RED") {
        tl.state = TrafficLightState::RED;
      } else if (class_id == "yellow" || class_id == "YELLOW" || class_id == "amber") {
        tl.state = TrafficLightState::YELLOW;
      } else if (class_id == "green" || class_id == "GREEN") {
        tl.state = TrafficLightState::GREEN;
      } else {
        tl.state = TrafficLightState::UNKNOWN;
      }
    } else {
      tl.state = TrafficLightState::UNKNOWN;
    }
  }

  /**
   * @brief Common upsert logic shared by all entity types.
   *
   * Resolves timestamp/frame_id from the object or message header,
   * pushes the detection onto history, and stores predictions.
   */
  template <typename EntityT>
  EntityT & upsertBase(
    std::unordered_map<int64_t, EntityT> & map,
    const world_model_msgs::msg::WorldObject & obj,
    const std_msgs::msg::Header & msg_header)
  {
    const auto & det = obj.detection;
    int64_t id = std::stoll(det.id);

    auto [it, inserted] = map.try_emplace(id, EntityT{});
    EntityT & entity = it->second;

    vision_msgs::msg::Detection3D stamped_det = det;
    if (det.header.stamp.sec == 0 && det.header.stamp.nanosec == 0) {
      stamped_det.header.stamp = msg_header.stamp;
    }
    if (det.header.frame_id.empty()) {
      stamped_det.header.frame_id = msg_header.frame_id;
    }
    entity.history.push_front(stamped_det);

    entity.predictions = obj.predictions;

    return entity;
  }
};

}  // namespace world_model

#endif  // WORLD_MODEL__PIPELINE__ENTITY_POPULATOR_HPP_
