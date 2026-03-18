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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_STAGE_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_STAGE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/logged_bt_node.hpp"

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
 * @class SetOvertakeStageAction
 * @brief SyncActionNode that writes the current overtake stage to the blackboard.
 */
class SetOvertakeStageAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetOvertakeStageAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::OvertakeStage>("value", "Stage to assign"),
      BT::OutputPort<types::OvertakeStage>("stage", "Assigned overtake stage"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto stage = ports::tryGet<types::OvertakeStage>(*this, "value");
    if (!ports::require(stage, "value", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("stage", *stage);
    RCLCPP_DEBUG_STREAM(logger(), "stage=" << types::toString(*stage) );
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__SET_OVERTAKE_STAGE_ACTION_HPP_
