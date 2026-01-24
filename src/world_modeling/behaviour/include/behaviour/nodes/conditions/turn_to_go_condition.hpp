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

#ifndef BEHAVIOUR__NODES__CONDITIONS__TURN_TO_GO_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__TURN_TO_GO_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>
#include <unordered_set>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class TurnToGoCondition
   * @brief BT condition node to check if all "first seen" objects have cleared the intersection.
   *
   * Inputs:
   *   - initial_objects: The objects seen when we first stopped at the stop sign.
   *   - current_objects: The current objects seen on the conflicting lanes.
   */
class TurnToGoCondition : public BT::ConditionNode
{
public:
  TurnToGoCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::DynamicObjectArray>("initial_objects"),
      BT::InputPort<types::DynamicObjectArray>("current_objects")};
  }

  BT::NodeStatus tick() override
  {
    types::DynamicObjectArray initial_objects;
    types::DynamicObjectArray current_objects;

    try {
      initial_objects = ports::get<types::DynamicObjectArray>(*this, "initial_objects");
      current_objects = ports::get<types::DynamicObjectArray>(*this, "current_objects");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (initial_objects.empty()) {
      return BT::NodeStatus::SUCCESS;
    }

    // Create a set of current object IDs for fast lookup
    std::unordered_set<int64_t> current_ids;
    for (const auto & obj : current_objects) {
      current_ids.insert(obj.id);
    }

    // Check if any of the initial objects are still present
    for (const auto & obj : initial_objects) {
      if (current_ids.count(obj.id) > 0) {
        // At least one object we are waiting for is still there
        return BT::NodeStatus::FAILURE;
      }
    }

    // All initial objects have cleared
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__CONDITIONS__TURN_TO_GO_CONDITION_HPP_
