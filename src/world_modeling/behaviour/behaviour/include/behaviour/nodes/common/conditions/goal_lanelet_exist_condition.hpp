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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_LANELET_EXIST_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_LANELET_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{
/**
 * @class GoalLaneletExistCondition
 * @brief ConditionNode to check whether a goal lanelet is available.
 */
class GoalLaneletExistCondition : public BT::ConditionNode
{
public:
  GoalLaneletExistCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("goal_lanelet")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GoalLaneletExist]: Missing " << port_name << " input" << std::endl;
    };

    auto goal_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "goal_lanelet");
    if (!ports::require(goal_lanelet, "goal_lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_LANELET_EXIST_CONDITION_HPP_
