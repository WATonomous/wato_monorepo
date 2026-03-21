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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_REG_ELEM_EGO_PRIORITY_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_REG_ELEM_EGO_PRIORITY_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"

namespace behaviour
{

/**
   * @class SetRegElemEgoPriorityAction
   * @brief Latches that ego has earned priority for the active regulatory element.
   */
class SetRegElemEgoPriorityAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetRegElemEgoPriorityAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<bool>("ego_priority")};
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_DEBUG_STREAM(logger(), "Setting ego priority for active regulatory element");
    setOutput("ego_priority", true);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_REG_ELEM_EGO_PRIORITY_ACTION_HPP_
