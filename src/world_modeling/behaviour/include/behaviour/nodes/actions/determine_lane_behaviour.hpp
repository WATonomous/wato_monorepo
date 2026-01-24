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

#ifndef BEHAVIOUR__NODES__ACTIONS__DETERMINE_LANE_BEHAVIOUR_HPP_
#define BEHAVIOUR__NODES__ACTIONS__DETERMINE_LANE_BEHAVIOUR_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class DetermineLaneBehaviourAction
   * @brief BT sync action node to determine the next lane behaviour based on current lane context and path.
   */
class DetermineLaneBehaviourAction : public BT::SyncActionNode
{
public:
  DetermineLaneBehaviourAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::CurrentLaneContextPtr>(
        "lanelet_context", "Pointer to current lane context containing ego position and current lanelet"),
      BT::InputPort<types::PathPtr>(
        "path", "Pointer to the planned path (GetRoute response) with lanelets and transitions"),
      BT::OutputPort<std::string>(
        "behaviour", "The determined behaviour: FOLLOW_LANE, LEFT_LANE_CHANGE, or RIGHT_LANE_CHANGE"),
      BT::OutputPort<types::LaneletPtr>("target_lanelet", "The next lanelet to transition to"),
      BT::OutputPort<std::string>("error_message", "Error description if the node fails"),
    };
  }

  BT::NodeStatus tick() override
  {
    types::CurrentLaneContextPtr lanelet_context;
    types::PathPtr path;

    try {
      lanelet_context = ports::getPtr<types::CurrentLaneContext>(*this, "lanelet_context");
      path = ports::getPtr<types::Path>(*this, "path");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (path->lanelets.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t current_lanelet_id = lanelet_context->current_lanelet.id;
    const auto & lanelets = path->lanelets;
    const auto & transitions = path->transitions;

    auto it = std::find_if(lanelets.begin(), lanelets.end(), [current_lanelet_id](const types::Lanelet & lanelet) {
      return lanelet.id == current_lanelet_id;
    });

    if (it == lanelets.end()) {
      return BT::NodeStatus::FAILURE;
    }

    const size_t current_index = static_cast<size_t>(std::distance(lanelets.begin(), it));
    const types::Lanelet & current_lanelet = lanelets[current_index];
    std::string behaviour = "follow_lane";
    size_t target_index = current_index;

    if (current_index + 1 < lanelets.size()) {
      target_index = current_index + 1;

      if (current_index < transitions.size()) {
        const uint8_t transition = transitions[current_index];

        // Transition types from GetRoute.srv:
        // TRANSITION_SUCCESSOR=0, TRANSITION_LEFT=1, TRANSITION_RIGHT=2
        constexpr uint8_t TRANSITION_SUCCESSOR = 0;
        constexpr uint8_t TRANSITION_LEFT = 1;
        constexpr uint8_t TRANSITION_RIGHT = 2;

        // TODO(wato): figure out how to register enums properly with btcpp
        if (transition == TRANSITION_LEFT && current_lanelet.can_change_left) {
          // behaviour = types::LaneBehaviour::LEFT_LANE_CHANGE;
          behaviour = "left_lane_change";
        } else if (transition == TRANSITION_RIGHT && current_lanelet.can_change_right) {
          // behaviour = types::LaneBehaviour::RIGHT_LANE_CHANGE;
          behaviour = "right_lane_change";
        } else if (transition == TRANSITION_SUCCESSOR) {
          // behaviour = types::LaneBehaviour::FOLLOW_LANE;
          behaviour = "follow_lane";
        } else if (transition == TRANSITION_LEFT || transition == TRANSITION_RIGHT) {
          // behaviour = types::LaneBehaviour::FOLLOW_LANE;
          behaviour = "follow_lane";
        } else {
          return BT::NodeStatus::FAILURE;
        }
      }
    }

    setOutput("behaviour", behaviour);
    setOutput("target_lanelet", std::make_shared<types::Lanelet>(lanelets[target_index]));

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__DETERMINE_LANE_BEHAVIOUR_HPP_
