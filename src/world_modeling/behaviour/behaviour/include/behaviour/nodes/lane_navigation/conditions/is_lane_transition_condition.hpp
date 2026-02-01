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

#ifndef BEHAVIOUR__IS_LANE_TRANSITION_CONDITION_HPP_
#define BEHAVIOUR__IS_LANE_TRANSITION_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{

  /**
   * @class IsLaneTransitionCondition
   * @brief BT condition node to check if a lane transition matches the expected value.
   *
   * Inputs:
   *   - value: The transition to check (LaneTransition)
   *   - expected: The expected transition (LaneTransition)
   */
  class IsLaneTransitionCondition : public BT::ConditionNode
  {
  public:
    IsLaneTransitionCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<types::LaneTransition>("transition", "The transition to check"),
          BT::InputPort<types::LaneTransition>("expected", "The expected transition")};
    }

    BT::NodeStatus tick() override
    {
      auto transition = ports::tryGet<types::LaneTransition>(*this, "transition");
      auto expected = ports::tryGet<types::LaneTransition>(*this, "expected");

      if (!transition)
      {
        std::cout << "[IsLaneTransitionCondition]: Missing transition input" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      if (!expected)
      {
        std::cout << "[IsLaneTransitionCondition]: Missing expected input" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      std::cout << "[IsLaneTransitionCondition]: Comparing transition=" << types::toString(transition.value())
                << ", expected=" << types::toString(expected.value()) << std::endl;
      return transition.value() == expected.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__IS_LANE_TRANSITION_CONDITION_HPP_
