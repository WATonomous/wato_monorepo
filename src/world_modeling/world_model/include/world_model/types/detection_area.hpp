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

#ifndef WORLD_MODEL__TYPES__DETECTION_AREA_HPP_
#define WORLD_MODEL__TYPES__DETECTION_AREA_HPP_

#include <cmath>
#include <string>

#include "world_model_msgs/msg/area_definition.hpp"

namespace world_model
{

/**
 * @brief Geometric area for occupancy detection.
 *
 * Supports three area types: Circle (center, radius), PartialCircle (center,
 * radius, angular bounds), and Rectangle (center, length, width).
 *
 * All coordinates are relative to a reference frame (typically base_link).
 */
class DetectionArea
{
public:
  enum class Type
  {
    Circle,
    PartialCircle,
    Rectangle
  };

  DetectionArea(
    const std::string & name,
    Type type,
    double center_x,
    double center_y,
    double radius = 0.0,
    double start_angle = 0.0,
    double end_angle = 0.0,
    double length = 0.0,
    double width = 0.0)
  : name_(name)
  , type_(type)
  , center_x_(center_x)
  , center_y_(center_y)
  , radius_(radius)
  , start_angle_(start_angle)
  , end_angle_(end_angle)
  , length_(length)
  , width_(width)
  {}

  /**
   * @brief Create DetectionArea from ROS message definition.
   */
  static DetectionArea fromMsg(const world_model_msgs::msg::AreaDefinition & msg)
  {
    Type type;
    switch (msg.area_type) {
      case world_model_msgs::msg::AreaDefinition::TYPE_CIRCLE:
        type = Type::Circle;
        break;
      case world_model_msgs::msg::AreaDefinition::TYPE_PARTIAL_CIRCLE:
        type = Type::PartialCircle;
        break;
      case world_model_msgs::msg::AreaDefinition::TYPE_RECTANGLE:
        type = Type::Rectangle;
        break;
      default:
        type = Type::Circle;
    }

    return DetectionArea(
      msg.name, type, msg.center_x, msg.center_y, msg.radius, msg.start_angle, msg.end_angle, msg.length, msg.width);
  }

  /**
   * @brief Check if a point (x, y) is inside this area.
   *
   * @param x X coordinate relative to area's reference frame
   * @param y Y coordinate relative to area's reference frame
   * @return true if point is inside the area
   */
  bool contains(double x, double y) const
  {
    switch (type_) {
      case Type::Circle:
        return containsCircle(x, y);
      case Type::PartialCircle:
        return containsPartialCircle(x, y);
      case Type::Rectangle:
        return containsRectangle(x, y);
      default:
        return false;
    }
  }

  /// @brief Returns the area name.
  const std::string & name() const
  {
    return name_;
  }

  /// @brief Returns the area type (Circle, PartialCircle, Rectangle).
  Type type() const
  {
    return type_;
  }

  /**
   * @brief Convert this DetectionArea to a ROS AreaDefinition message.
   *
   * @return Populated AreaDefinition message with all geometric parameters.
   */
  world_model_msgs::msg::AreaDefinition toMsg() const
  {
    world_model_msgs::msg::AreaDefinition msg;
    msg.name = name_;
    msg.center_x = center_x_;
    msg.center_y = center_y_;
    msg.radius = radius_;
    msg.start_angle = start_angle_;
    msg.end_angle = end_angle_;
    msg.length = length_;
    msg.width = width_;

    switch (type_) {
      case Type::Circle:
        msg.area_type = world_model_msgs::msg::AreaDefinition::TYPE_CIRCLE;
        break;
      case Type::PartialCircle:
        msg.area_type = world_model_msgs::msg::AreaDefinition::TYPE_PARTIAL_CIRCLE;
        break;
      case Type::Rectangle:
        msg.area_type = world_model_msgs::msg::AreaDefinition::TYPE_RECTANGLE;
        break;
    }

    return msg;
  }

private:
  /// @brief Circle containment test: checks if (x,y) is within the radius.
  bool containsCircle(double x, double y) const
  {
    double dx = x - center_x_;
    double dy = y - center_y_;
    double dist_sq = dx * dx + dy * dy;
    return dist_sq <= radius_ * radius_;
  }

  /// @brief Partial circle containment test: checks radius and angular bounds.
  bool containsPartialCircle(double x, double y) const
  {
    double dx = x - center_x_;
    double dy = y - center_y_;
    double dist_sq = dx * dx + dy * dy;

    // Check radius
    if (dist_sq > radius_ * radius_) {
      return false;
    }

    // Check angle (0 = forward/+x, CCW positive)
    double angle = std::atan2(dy, dx);

    // Normalize angles to [0, 2*PI)
    auto normalize = [](double a) {
      while (a < 0) {
        a += 2 * M_PI;
      }
      while (a >= 2 * M_PI) {
        a -= 2 * M_PI;
      }
      return a;
    };

    double norm_angle = normalize(angle);
    double norm_start = normalize(start_angle_);
    double norm_end = normalize(end_angle_);

    // Handle wrap-around case
    if (norm_start <= norm_end) {
      return norm_angle >= norm_start && norm_angle <= norm_end;
    } else {
      // Wrap-around: angle is valid if >= start OR <= end
      return norm_angle >= norm_start || norm_angle <= norm_end;
    }
  }

  /// @brief Rectangle containment test: checks axis-aligned half-extents.
  bool containsRectangle(double x, double y) const
  {
    // Transform point to rectangle's local frame (centered at rectangle center)
    double local_x = x - center_x_;
    double local_y = y - center_y_;

    // Rectangle is axis-aligned in local frame
    double half_length = length_ / 2.0;
    double half_width = width_ / 2.0;

    return std::abs(local_x) <= half_length && std::abs(local_y) <= half_width;
  }

  std::string name_;
  Type type_;
  double center_x_;
  double center_y_;
  double radius_;
  double start_angle_;
  double end_angle_;
  double length_;
  double width_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__TYPES__DETECTION_AREA_HPP_
