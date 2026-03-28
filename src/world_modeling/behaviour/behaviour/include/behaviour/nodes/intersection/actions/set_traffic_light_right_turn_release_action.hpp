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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_TRAFFIC_LIGHT_RIGHT_TURN_RELEASE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_TRAFFIC_LIGHT_RIGHT_TURN_RELEASE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{

class SetTrafficLightRightTurnReleaseAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetTrafficLightRightTurnReleaseAction(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("release_latched"),
      BT::OutputPort<bool>("out_release_latched"),
    };
  }

  BT::NodeStatus tick() override
  {
    const bool already_latched = ports::tryGet<bool>(*this, "release_latched").value_or(false);
    if (!already_latched) {
      RCLCPP_DEBUG(logger(), "latching traffic light right-turn release");
    }

    setOutput("out_release_latched", true);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_TRAFFIC_LIGHT_RIGHT_TURN_RELEASE_ACTION_HPP_
