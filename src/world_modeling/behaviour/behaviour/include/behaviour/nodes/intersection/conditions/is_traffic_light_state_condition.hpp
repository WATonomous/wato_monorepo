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

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
   * @class IsTrafficLightStateCondition
   * @brief ConditionNode to compare traffic light state with expected state.
   */
class IsTrafficLightStateCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  IsTrafficLightStateCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
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
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto state = ports::tryGet<std::string>(*this, "traffic_light_state");
    if (!ports::require(state, "traffic_light_state", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto expected = ports::tryGet<std::string>(*this, "expected");
    if (!ports::require(expected, "expected", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    return (*state == *expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__IS_TRAFFIC_LIGHT_STATE_CONDITION_HPP_
