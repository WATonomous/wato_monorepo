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

#ifndef BEHAVIOUR__NODES__CONDITIONS__REG_ELEM_TYPE_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__REG_ELEM_TYPE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class RegElemTypeCondition
   * @brief BT condition node to check if the selected regulatory element matches an expected type.
   *
   * Inputs:
   *   - value: The regulatory element to check (RegulatoryElementPtr)
   *   - expected: The expected type (RegElemType enum)
   */
class RegElemTypeCondition : public BT::ConditionNode
{
public:
  RegElemTypeCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::RegulatoryElementPtr>("value", "The element to check"),
      BT::InputPort<types::RegElemType>("expected", "The expected type")};
  }

  BT::NodeStatus tick() override
  {
    types::RegulatoryElementPtr elem;
    types::RegElemType expected;

    try {
      elem = ports::getPtr<types::RegulatoryElement>(*this, "value");
      expected = ports::get<types::RegElemType>(*this, "expected");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    return (elem->type == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__CONDITIONS__REG_ELEM_TYPE_CONDITION_HPP_
