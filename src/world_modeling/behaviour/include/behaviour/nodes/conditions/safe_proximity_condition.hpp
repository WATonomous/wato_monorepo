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

#ifndef BEHAVIOUR__NODES__CONDITIONS__SAFE_PROXIMITY_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__SAFE_PROXIMITY_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cmath>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class SafeProximityCondition
   * @brief BT node to check if any dynamic objects are within a specified radius of the car.
   */
class SafeProximityCondition : public BT::ConditionNode
{
public:
  SafeProximityCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::DynamicObjectArray>("objects", "List of objects to check"),
      BT::InputPort<types::PointPtr>("current_point", "Current point of the ego vehicle"),
      BT::InputPort<double>("radius", 15.0, "Safe radius in meters")};
  }

  BT::NodeStatus tick() override
  {
    types::DynamicObjectArray objects;
    types::PointPtr current_point;
    double radius;

    try {
      objects = ports::get<types::DynamicObjectArray>(*this, "objects");
      current_point = ports::getPtr<geometry_msgs::msg::Point>(*this, "current_point");
      radius = ports::get<double>(*this, "radius");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    const double radius_sq = std::pow(radius, 2);

    for (const auto & obj : objects) {
      double dx = obj.pose.position.x - current_point->x;
      double dy = obj.pose.position.y - current_point->y;
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq < radius_sq) {
        // Found an object within the radius, so it's NOT safe
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__CONDITIONS__SAFE_PROXIMITY_CONDITION_HPP_
