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

#ifndef BEHAVIOUR__NODES__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_EXIST_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
  /**
   * @class ActiveTrafficControlElementExistCondition
   * @brief Checks whether a latched traffic-control element is present.
   */
  class ActiveTrafficControlElementExistCondition : public BT::ConditionNode
  {
  public:
    ActiveTrafficControlElementExistCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("active_traffic_control_element"),
      };
    }

    BT::NodeStatus tick() override
    {
      auto active = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "active_traffic_control_element");

      std::cout << "[ActiveTrafficControlElementExistCondition]: active_traffic_control_element "
                << (active && *active ? "exists." : "does not exist.") << std::endl;
      return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_EXIST_CONDITION_HPP_