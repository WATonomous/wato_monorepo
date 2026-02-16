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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
  /**
   * @class ClearActiveTrafficControlElementAction
   * @brief SyncActionNode to clear the active traffic control element.
   */
  class ClearActiveTrafficControlElementAction : public BT::SyncActionNode
  {
  public:
    ClearActiveTrafficControlElementAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("active_traffic_control_element"),
          BT::OutputPort<int64_t>("active_traffic_control_lanelet_id"),
          BT::OutputPort<int64_t>("active_traffic_control_element_id"),
      };
    }

    BT::NodeStatus tick() override
    {
      std::cout << "[ClearActiveTrafficControlElementAction]: Clearing active traffic control element "
                << "and associated lanelet/element IDs" << std::endl;
      lanelet_msgs::msg::RegulatoryElement::SharedPtr cleared = nullptr;
      setOutput("active_traffic_control_element", cleared);
      setOutput("active_traffic_control_lanelet_id", static_cast<int64_t>(0));
      setOutput("active_traffic_control_element_id", static_cast<int64_t>(0));
      return BT::NodeStatus::SUCCESS;
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_ACTIVE_TRAFFIC_CONTROL_ELEMENT_ACTION_HPP_
