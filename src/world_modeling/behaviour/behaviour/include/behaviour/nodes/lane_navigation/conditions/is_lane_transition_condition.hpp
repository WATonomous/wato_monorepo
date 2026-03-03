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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__IS_LANE_TRANSITION_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__IS_LANE_TRANSITION_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{

/**
   * @class IsLaneTransitionCondition
   * @brief ConditionNode to compare lane transition with expected value.
   */
class IsLaneTransitionCondition : public BT::ConditionNode
{
public:
  IsLaneTransitionCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::LaneTransition>("lane_transition", "The transition to check"),
      BT::InputPort<types::LaneTransition>("expected", "The expected transition")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[IsLaneTransitionCondition]: Missing " << port_name << " input" << std::endl;
    };

    auto lane_transition = ports::tryGet<types::LaneTransition>(*this, "lane_transition");
    if (!ports::require(lane_transition, "lane_transition", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto expected = ports::tryGet<types::LaneTransition>(*this, "expected");
    if (!ports::require(expected, "expected", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "[IsLaneTransitionCondition]: Comparing transition=" << types::toString(lane_transition.value())
              << ", expected=" << types::toString(expected.value()) << std::endl;
    return lane_transition.value() == expected.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__IS_LANE_TRANSITION_CONDITION_HPP_
