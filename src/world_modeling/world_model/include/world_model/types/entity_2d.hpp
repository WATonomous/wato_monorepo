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

#ifndef WORLD_MODEL__TYPES__ENTITY_2D_HPP_
#define WORLD_MODEL__TYPES__ENTITY_2D_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "world_model/types/entity.hpp"

namespace world_model
{

/**
 * @brief Base class for 2D tracked entities (image-based detections).
 *
 * Stores detection history with most recent at front.
 */
class Entity2D : public Entity
{
public:
  // Detection history (front = most recent)
  std::deque<vision_msgs::msg::Detection2D> history;

  // ─────────────────────────────────────────────────────────────────────────
  // Convenience accessors
  // ─────────────────────────────────────────────────────────────────────────

  bool empty() const
  {
    return history.empty();
  }

  const vision_msgs::msg::Detection2D & detection() const
  {
    return history.front();
  }

  int64_t id() const
  {
    return std::stoll(history.front().id);
  }

  rclcpp::Time timestamp() const
  {
    return rclcpp::Time(history.front().header.stamp);
  }

  const std::string & frameId() const
  {
    return history.front().header.frame_id;
  }

  const vision_msgs::msg::BoundingBox2D & bbox() const
  {
    return history.front().bbox;
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// 2D Entity Subclasses
// ═══════════════════════════════════════════════════════════════════════════

enum class TrafficLightState : uint8_t
{
  UNKNOWN = 0,
  RED = 1,
  YELLOW = 2,
  GREEN = 3
};

/**
 * @brief Traffic light entity with state interpretation.
 *
 * Extends Entity2D with traffic light specific fields:
 * interpreted state, confidence, and lanelet regulatory element ID.
 */
class TrafficLight : public Entity2D
{
public:
  EntityType type() const override
  {
    return EntityType::TRAFFIC_LIGHT;
  }

  // Interpreted state (parsed from detection.results)
  TrafficLightState state{TrafficLightState::UNKNOWN};
  float confidence{0.0f};

  // Lanelet regulatory element ID (links to map)
  int64_t reg_elem_id{-1};
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_2D_HPP_
