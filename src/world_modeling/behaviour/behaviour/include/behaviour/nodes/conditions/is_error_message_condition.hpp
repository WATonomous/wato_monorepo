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

#ifndef BEHAVIOUR__IS_ERROR_MESSAGE_CONDITION_HPP_
#define BEHAVIOUR__IS_ERROR_MESSAGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class IsErrorMessageCondition
   * @brief Compares an error message against an expected value.
   */
class IsErrorMessageCondition : public BT::ConditionNode
{
public:
  IsIsErrorMessageCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("msg"), BT::InputPort<std::string>("expected")};
  }

  /**
     * @brief Compares current error message against expected value.
     * @return SUCCESS if they match, FAILURE otherwise.
     */
  BT::NodeStatus tick() override
  {
    auto msg = ports::tryGet<std::string>(*this, "msg");
    auto expected = ports::tryGet<std::string>(*this, "expected");

    if (!msg || !expected) {
      std::cout << "[IsErrorMessageCondition]: Missing msg or expected input" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (msg == expected) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__IS_MANEUVER_CONDITION_HPP_
