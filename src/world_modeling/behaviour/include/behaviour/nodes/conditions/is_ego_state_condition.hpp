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

#ifndef BEHAVIOUR__IS_EGO_STATE_CONDITION_HPP_
#define BEHAVIOUR__IS_EGO_STATE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "bvehicle_states.hpp"

namespace wato::world_modeling::behaviour
{
class IsEgoStateCondition : public BT::ConditionNode
{
public:
  IsEgoStateCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<VehicleState>("state"), BT::InputPort<VehicleState>("expected")};
  }

  BT::NodeStatus tick() override
  {
    // TODO(wato): maybe make a port validator to validate all ports for a node in the future
    auto state = getInput<VehicleState>("state");  // from a future subscriber node?
    auto expected = getInput<VehicleState>("expected");

    // validate port
    if (!state || !expected) {
      return BT::NodeStatus::FAILURE;
    }

    // logic
    if (state.value() == expected.value()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace wato::world_modeling::behaviour
#endif
