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

#include <behaviortree_cpp/bt_factory.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// ROS message types
#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/srv/get_route.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"


namespace behaviour::types
{
// blackboard types

// Geometry
using PointPtr = std::shared_ptr<geometry_msgs::msg::Point>;

// Dynamic Objects
using DynamicObject = world_model_msgs::msg::DynamicObject;
using DynamicObjectArray = std::vector<DynamicObject>;

// Lanelet types
using Lanelet = lanelet_msgs::msg::Lanelet;
using LaneletPtr = std::shared_ptr<Lanelet>;
using LaneletArray = std::vector<Lanelet>;
using LaneletArrayPtr = std::shared_ptr<LaneletArray>;

using CurrentLaneContext = lanelet_msgs::msg::CurrentLaneContext;
using CurrentLaneContextPtr = std::shared_ptr<CurrentLaneContext>;

// Route/Path types
using Path = lanelet_msgs::srv::GetRoute::Response;
using PathPtr = std::shared_ptr<Path>;

enum class RegElemType
{
  TRAFFIC_LIGHT,
  STOP_SIGN,
  YIELD,
  NONE
};

struct RegulatoryElement
{
  int64_t id;
  int64_t stop_line_id;
  RegElemType type;
  double distance;
};

using RegulatoryElementPtr = std::shared_ptr<RegulatoryElement>;
using RegulatoryElementArray = std::vector<RegulatoryElement>;
using RegulatoryElementArrayPtr = std::shared_ptr<RegulatoryElementArray>;

/**
   * @brief Lane behaviour enum for specifying driving maneuvers.
   *
   * Can be used in XML like: behaviour="follow_lane"
   */

// TODO(wato): Figure out how to register enums with btcpp
// enum class LaneBehaviour
// {
//   FOLLOW_LANE,
//   LEFT_LANE_CHANGE,
//   RIGHT_LANE_CHANGE,
//   STANDBY
// };

// inline const char *toString(LaneBehaviour b)
// {
//   switch (b)
//   {
//   case LaneBehaviour::LEFT_LANE_CHANGE:
//     return "left_lane_change";
//   case LaneBehaviour::RIGHT_LANE_CHANGE:
//     return "right_lane_change";
//   case LaneBehaviour::STANDBY:
//     return "standby";
//   case LaneBehaviour::FOLLOW_LANE:
//   default:
//     return "follow_lane";
//   }
// }

}  // namespace behaviour::types

// custom xml types
namespace BT
{
/**
   * @brief Convert string to RegElemType enum.
   *
   * Allows XML like: <Node expected="stop_sign"/>
   */
template <>
inline behaviour::types::RegElemType convertFromString(StringView str)
{
  if (str == "traffic_light") {
    return behaviour::types::RegElemType::TRAFFIC_LIGHT;
  }
  if (str == "stop_sign") {
    return behaviour::types::RegElemType::STOP_SIGN;
  }
  if (str == "yield") {
    return behaviour::types::RegElemType::YIELD;
  }
  return behaviour::types::RegElemType::NONE;
}

}  // namespace BT

#endif  // BEHAVIOUR__UTILS__TYPES_HPP_
