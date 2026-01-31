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

#ifndef BEHAVIOUR__NODES__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class GetTrafficLightStateAction
   * @brief BT node to get the current state of a traffic light.
   *
   * TODO(wato): Implement this node. It needs to query the world model or a
   * dedicated traffic light status topic/service.
   */
class GetTrafficLightStateAction : public BT::SyncActionNode
{
public:
  GetTrafficLightStateAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int64_t>("traffic_light_id"),
      BT::OutputPort<std::string>("state", "Current state: red, yellow, green, unknown")};
  }

  BT::NodeStatus tick() override
  {
    // TODO(wato): Implement actual status lookup
    setOutput("state", std::string("unknown"));
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_
