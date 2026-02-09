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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_CAN_PROCEED_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_CAN_PROCEED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class StopSignCanProceedCondition
 * @brief ConditionNode to check whether stop-sign priority cars have cleared.
 */
class StopSignCanProceedCondition : public BT::ConditionNode
{
public:
  StopSignCanProceedCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("priority_ids"),
      BT::InputPort<std::vector<std::string>>("current_ids"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto priority = ports::tryGet<std::vector<std::string>>(*this, "priority_ids");
    auto current = ports::tryGet<std::vector<std::string>>(*this, "current_ids");

    if (!priority) {
      std::cout << "[StopSignCanProceed] Missing priority_ids" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!current) {
      std::cout << "[StopSignCanProceed] Missing current_ids" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::unordered_set<std::string> current_set;
    current_set.reserve(current->size());
    for (const auto & id : *current) {
      current_set.insert(id);
    }

    for (const auto & id : *priority) {
      if (current_set.find(id) != current_set.end()) {
        std::cout << "[StopSignCanProceed] Blocked (priority car still present: " << id << ")" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }

    std::cout << "[StopSignCanProceed] Clear (no priority cars present)" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_CAN_PROCEED_CONDITION_HPP_
