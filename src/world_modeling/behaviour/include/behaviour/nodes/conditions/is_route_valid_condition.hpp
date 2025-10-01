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

#ifndef BEHAVIOUR__IS_ROUTE_VALID_CONDITION_HPP_
#define BEHAVIOUR__IS_ROUTE_VALID_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <memory>
#include <vector>

// srv
#include "world_modeling_msgs/msg/lanelet.hpp"

namespace behaviour
{
/**
   * @class IsRouteValidCondition
   * @brief Checks if a valid global route exists on the blackboard ().
   */
class IsRouteValidCondition : public BT::ConditionNode
{
public:
  IsRouteValidCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort < std::shared_ptr<std::vector<world_modeling_msgs::msg::Lanelet>>;
      > ("global_path", "Pointer to the global route lanelets")
    };
  }

  /**
     * @brief Checks if a route is valid and non-empty.
     * @return SUCCESS if route exists, FAILURE otherwise.
     */
  BT::NodeStatus tick() override
  {
    auto global_path = getInput<std::shared_ptr<std::vector<world_modeling_msgs::msg::Lanelet>>>("global_path");

    // validate port
    if (!global_path) {
      return BT::NodeStatus::FAILURE;
    }

    if (!global_path.value() || global_path.value()->empty()) {
      return BT::NodeStatus::FAILURE;
    }
    // TODO(wato): needs to check if car is off track

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__IS_ROUTE_VALID_CONDITION_HPP_
