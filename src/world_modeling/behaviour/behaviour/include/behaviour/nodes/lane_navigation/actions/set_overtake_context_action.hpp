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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/types.hpp"

namespace behaviour
{
/**
 * @class SetOvertakeContextAction
 * @brief Derives diverge/merge transitions and behaviours from the chosen diverge direction.
 */
class SetOvertakeContextAction : public BT::SyncActionNode
{
public:
  SetOvertakeContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::LaneTransition>("in_diverge_lane_transition"),
      BT::OutputPort<types::LaneTransition>("out_diverge_lane_transition"),
      BT::OutputPort<types::LaneTransition>("out_merge_lane_transition"),
      BT::OutputPort<std::string>("out_behaviour_diverge"),
      BT::OutputPort<std::string>("out_behaviour_merge"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[SetOvertakeContext]: Missing " << port_name << " input" << std::endl;
    };

    auto diverge_transition = ports::tryGet<types::LaneTransition>(*this, "in_diverge_lane_transition");
    if (!ports::require(diverge_transition, "in_diverge_lane_transition", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    types::LaneTransition merge_transition;
    std::string diverge_behaviour;
    std::string merge_behaviour;

    switch (*diverge_transition) {
      case types::LaneTransition::LEFT:
        merge_transition = types::LaneTransition::RIGHT;
        diverge_behaviour = "overtake: pull out left";
        merge_behaviour = "overtake: merge back right";
        break;
      case types::LaneTransition::RIGHT:
        merge_transition = types::LaneTransition::LEFT;
        diverge_behaviour = "overtake: pull out right";
        merge_behaviour = "overtake: merge back left";
        break;
      case types::LaneTransition::SUCCESSOR:
      default:
        std::cout << "[SetOvertakeContext]: Invalid diverge transition "
                  << types::toString(*diverge_transition) << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("out_diverge_lane_transition", *diverge_transition);
    setOutput("out_merge_lane_transition", merge_transition);
    setOutput("out_behaviour_diverge", diverge_behaviour);
    setOutput("out_behaviour_merge", merge_behaviour);

    std::cout << "[SetOvertakeContext]: diverge=" << types::toString(*diverge_transition)
              << ", merge=" << types::toString(merge_transition) << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_CONTEXT_ACTION_HPP_
