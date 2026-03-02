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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_ERROR_MESSAGE_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_ERROR_MESSAGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
 * @class IsErrorMessageCondition
 * @brief ConditionNode to compare an error message with an expected value.
 */
class IsErrorMessageCondition : public BT::ConditionNode
{
public:
  IsErrorMessageCondition(const std::string & name, const BT::NodeConfig & config)
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
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[IsErrorMessageCondition]: Missing " << port_name << " input" << std::endl;
    };

    auto msg = ports::tryGet<std::string>(*this, "msg");
    if (!ports::require(msg, "msg", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto expected = ports::tryGet<std::string>(*this, "expected");
    if (!ports::require(expected, "expected", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "[IsErrorMessageCondition]: Comparing msg='" << *msg << "' to expected='" << *expected << "'"
              << std::endl;

    if (msg == expected) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_ERROR_MESSAGE_CONDITION_HPP_
