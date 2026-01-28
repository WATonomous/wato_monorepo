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

#ifndef BEHAVIOUR__HAS_GOAL_CONDTION_HPP_
#define BEHAVIOUR__HAS_GOAL_CONDTION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "geometry_msgs/msg/point.hpp"

namespace behaviour
{
class HasGoalCondition : public BT::ConditionNode
{
public:
  HasGoalCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("goal_point")};
  }

  BT::NodeStatus tick() override
  {
    auto gp = ports::tryGetPtr<geometry_msgs::msg::Point>(*this, "goal_point");
    if (gp == nullptr) {
      std::cout << "HasGoal: Missing goal point." << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour
#endif
