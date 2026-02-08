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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__IS_TRAFFIC_LIGHT_STATE_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__IS_TRAFFIC_LIGHT_STATE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class IsTrafficLightStateCondition
 * @brief ConditionNode to compare traffic light state with expected state.
 */
class IsTrafficLightStateCondition : public BT::ConditionNode
{
public:
  IsTrafficLightStateCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("traffic_light_state"),
      BT::InputPort<std::string>("expected"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto state = ports::tryGet<std::string>(*this, "traffic_light_state");
    auto expected = ports::tryGet<std::string>(*this, "expected");

    if (!state) {
      std::cerr << "[IsTrafficLightState] Missing traffic_light_state." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!expected) {
      std::cerr << "[IsTrafficLightState] Missing expected." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // case-insensitive comparison
    auto norm = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      return s;
    };

    const std::string s_norm = norm(*state);
    const std::string e_norm = norm(*expected);

    std::cout << "[IsTrafficLightState]: Comparing msg='" << s_norm << "' to expected='" << e_norm << "'" << std::endl;

    return (s_norm == e_norm) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__IS_TRAFFIC_LIGHT_STATE_CONDITION_HPP_
