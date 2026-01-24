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

#ifndef BEHAVIOUR__GOAL_REACHED_CONDTION_HPP_
#define BEHAVIOUR__GOAL_REACHED_CONDTION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
class GoalReachedConditon : public BT::ConditionNode
{
public:
  GoalReachedConditon(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("current_point"),
      BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("goal_point"),
      BT::InputPort<double>("threshold_m")};
  };

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Point::SharedPtr gp;
    geometry_msgs::msg::Point::SharedPtr cp;

    try {
      gp = ports::getPtr<geometry_msgs::msg::Point>(*this, "goal_point");
      cp = ports::getPtr<geometry_msgs::msg::Point>(*this, "current_point");
    } catch (const BT::RuntimeError & e) {
      std::cout << "GoalReachedCondition: Missing goal point or current point." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    double threshold = ports::get<double>(*this, "threshold_m");

    // Calculate 2D distance (ignoring z)
    double dx = gp->x - cp->x;
    double dy = gp->y - cp->y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Check if within threshold
    if (distance <= threshold) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour
#endif
