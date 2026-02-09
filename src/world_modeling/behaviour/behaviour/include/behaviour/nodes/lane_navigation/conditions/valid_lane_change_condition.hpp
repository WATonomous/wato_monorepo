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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__VALID_LANE_CHANGE_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__VALID_LANE_CHANGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour
{
  /**
   * @class ValidLaneChangeCondition
   * @brief ConditionNode to validate whether a lane change is permitted.
   *
   * Logic:
   * - Reject when transition or lane context is missing.
   * - Reject when ego is currently in an intersection.
   * - For `LEFT`, return `SUCCESS` only if `can_change_left` is true.
   * - For `RIGHT`, return `SUCCESS` only if `can_change_right` is true.
   * - Return `FAILURE` for unsupported transition values.
   *
   * Assumptions:
   * - Lanelet `can_change_left/right` flags represent legal lane-change policy.
   * - Valid lane change does not mean its a safe lane change.
   * - Intersection constraint is a hard stop for lane-change validity.
   */
  class ValidLaneChangeCondition : public BT::ConditionNode
  {
  public:
    ValidLaneChangeCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<types::LaneTransition>("lane_transition", "The transition to check"),
          BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx", "The expected transition"),
      };
    }

    BT::NodeStatus tick() override
    {
      const auto missing_input_callback = [&](const char *port_name)
      { std::cout << "[ValidLaneChange]: Missing " << port_name << " input" << std::endl; };

      auto lane_transition = ports::tryGet<types::LaneTransition>(*this, "lane_transition");
      if (!ports::require(lane_transition, "lane_transition", missing_input_callback))
        return BT::NodeStatus::FAILURE;
      auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
      if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback))
        return BT::NodeStatus::FAILURE;

      // Check if in intersection
      lanelet_msgs::msg::Lanelet current_lanelet = lane_ctx->current_lanelet;
      if (current_lanelet.is_intersection)
      {
        std::cout << "[ValidLaneChange]: Not valid, currently in intersection" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      bool can_change_left = current_lanelet.can_change_left;
      bool can_change_right = current_lanelet.can_change_right;

      switch (lane_transition.value())
      {
      // handles left lane change
      case types::LaneTransition::LEFT:
        if (can_change_left)
        {
          std::cout << "[ValidLaneChange]: Valid left lane change" << std::endl;
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          std::cout << "[ValidLaneChange]: Not valid, cannot change left" << std::endl;
          return BT::NodeStatus::FAILURE;
        }
      // handles right lane change
      case types::LaneTransition::RIGHT:
        if (can_change_right)
        {
          std::cout << "[ValidLaneChange]: Valid right lane change" << std::endl;
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          std::cout << "[ValidLaneChange]: Not valid, cannot change right" << std::endl;
          return BT::NodeStatus::FAILURE;
        }
      default:
        std::cout << "[ValidLaneChange]: Invalid transition type" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__VALID_LANE_CHANGE_CONDITION_HPP_
