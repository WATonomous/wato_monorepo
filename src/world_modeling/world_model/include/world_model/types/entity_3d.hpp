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

#include <deque>
#include <string>
#include <vector>

#include "prediction_msgs/msg/prediction.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "world_model/types/entity.hpp"

namespace world_model
{

/**
 * @brief Base class for 3D tracked entities (mobile objects).
 *
 * Stores detection history with most recent at front.
 * Can hold predicted future paths from Prediction node.
 */
class Entity3D : public Entity
{
public:
  // Detection history (front = most recent)
  std::deque<vision_msgs::msg::Detection3D> history;

  // Predicted future paths (from Prediction node)
  std::vector<prediction_msgs::msg::Prediction> predictions;

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

// 3D Entity Subclasses (extend later if needed)

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

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__ENTITY_3D_HPP_
