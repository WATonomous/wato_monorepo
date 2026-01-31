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

#ifndef BEHAVIOUR__GOAL_UPDATED_CONDTION_HPP_
#define BEHAVIOUR__GOAL_UPDATED_CONDTION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
class GoalUpdatedCondition : public BT::ConditionNode
{
public:
  GoalUpdatedCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("goal_updated")};
  };

  BT::NodeStatus tick() override
  {
    auto goal_updated = ports::get<bool>(*this, "goal_updated");

    if (goal_updated) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour
#endif
