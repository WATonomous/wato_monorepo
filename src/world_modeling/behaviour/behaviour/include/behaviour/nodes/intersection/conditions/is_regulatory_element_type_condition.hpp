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

#ifndef BEHAVIOUR__NODES__CONDITIONS__IS_REGULATORY_ELEMENT_TYPE_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__IS_REGULATORY_ELEMENT_TYPE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
  /**
   * @class IsRegulatoryElementTypeCondition
   * @brief Checks if the reg element has the expected subtype.
   */
  class IsRegulatoryElementTypeCondition : public BT::ConditionNode
  {
  public:
    IsRegulatoryElementTypeCondition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("reg_elem"),
          BT::InputPort<std::string>("expected"),
      };
    }

    BT::NodeStatus tick() override
    {
      auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "reg_elem");
      auto expected = ports::tryGet<std::string>(*this, "expected");

      if (!reg_elem)
      {
        std::cerr << "[IsRegulatoryElementTypeCondition] Missing reg_elem." << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      if (!expected)
      {
        std::cerr << "[IsRegulatoryElementTypeCondition] Missing expected." << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      std::cout << "[IsRegulatoryElementTypeCondition]: Comparing msg='" << reg_elem->subtype << "' to expected='" << expected.value() << "'" << std::endl;
      return (reg_elem->subtype == expected.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__CONDITIONS__IS_REGULATORY_ELEMENT_TYPE_CONDITION_HPP_