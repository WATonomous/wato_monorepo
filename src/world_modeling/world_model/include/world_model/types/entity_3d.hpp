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

#ifndef WORLD_MODEL__TYPES__ENTITY_3D_HPP_
#define WORLD_MODEL__TYPES__ENTITY_3D_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "world_model_msgs/msg/prediction.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "world_model/types/entity_type.hpp"

namespace world_model
{

/**
 * @brief Base class for all tracked entities.
 *
 * Stores detection history with most recent at front.
 * Can hold predicted future paths and lanelet context.
 */
class Entity3D
{
public:
  virtual ~Entity3D() = default;
  virtual EntityType type() const = 0;

  // Lanelet context (enriched by LaneletEnricher)
  std::optional<int64_t> lanelet_id;
  // Detection history (front = most recent)
  std::deque<vision_msgs::msg::Detection3D> history;

  // Predicted future paths (from Prediction node)
  std::vector<world_model_msgs::msg::Prediction> predictions;

  bool empty() const
  {
    return history.empty();
  }

  const vision_msgs::msg::Detection3D & detection() const
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

  const geometry_msgs::msg::Pose & pose() const
  {
    return history.front().bbox.center;
  }

  const geometry_msgs::msg::Vector3 & size() const
  {
    return history.front().bbox.size;
  }

  const vision_msgs::msg::BoundingBox3D & bbox() const
  {
    return history.front().bbox;
  }
};

// 3D Entity Subclasses

class Unknown : public Entity3D
{
public:
  EntityType type() const override
  {
    return EntityType::UNKNOWN;
  }
};

class Car : public Entity3D
{
public:
  EntityType type() const override
  {
    return EntityType::CAR;
  }

  // Future: turn signals, brake lights, etc.
};

class Human : public Entity3D
{
public:
  EntityType type() const override
  {
    return EntityType::HUMAN;
  }

  // Future: skeleton keypoints, gesture, etc.
};

class Bicycle : public Entity3D
{
public:
  EntityType type() const override
  {
    return EntityType::BICYCLE;
  }

  // Future: rider info, cargo, etc.
};

class Motorcycle : public Entity3D
{
public:
  EntityType type() const override
  {
    return EntityType::MOTORCYCLE;
  }

  // Future: rider count, helmet detection, etc.
};

// TrafficLight (3D entity with state interpretation)

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
 * Extends Entity3D with traffic light specific fields:
 * interpreted state, confidence, and lanelet regulatory element ID.
 */
class TrafficLight : public Entity3D
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

#endif  // WORLD_MODEL__TYPES__ENTITY_3D_HPP_
