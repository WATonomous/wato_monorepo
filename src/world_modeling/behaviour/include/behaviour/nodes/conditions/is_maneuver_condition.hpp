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

#ifndef BEHAVIOUR__IS_MANEUVER_CONDITION_HPP_
#define BEHAVIOUR__IS_MANEUVER_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

namespace behaviour
{
/**
   * @class IsManeuverCondition
   * @brief Checks if the current maneuver matches an expected type.
   */
class IsManeuverCondition : public BT::ConditionNode
{
public:
  IsManeuverCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("maneuver", "Current maneuver from ComputeManeuver"),
      BT::InputPort<std::string>("expected", "Expected maneuver type to match")};
  }

  /**
     * @brief Compares current maneuver against expected value.
     * @return SUCCESS if they match, FAILURE otherwise.
     */
  BT::NodeStatus tick() override
  {
    auto maneuver = getInput<std::string>("maneuver");
    auto expected = getInput<std::string>("expected");

    // validate port
    if (!maneuver || !expected) {
      return BT::NodeStatus::FAILURE;
    }

    if (maneuver.value() == expected.value()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__IS_MANEUVER_CONDITION_HPP_
