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

#ifndef BEHAVIOUR__NODES__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
 * @class ClearActiveTrafficControlElementAction
 * @brief Clears the latched active traffic-control element.
 */
class ClearActiveTrafficControlElementAction : public BT::SyncActionNode
{
public:
  ClearActiveTrafficControlElementAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("in_active_traffic_control_element"),
      BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("out_active_traffic_control_element"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "in_active_traffic_control_element");
    if (!reg_elem) {
      std::cerr << "[ClearActiveTrafficControlElementAction] Missing in_active_traffic_control_element." << std::endl;
    }
    
    std::cout << "[ClearActiveTrafficControlElementAction]: Clearing active traffic control element" << std::endl;
    setOutput("out_active_traffic_control_element", reg_elem.reset());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_
