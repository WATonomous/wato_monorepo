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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_LANELET_TURN_DIRECTION_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_LANELET_TURN_DIRECTION_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <iostream>
#include <memory>
#include <string>

#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{
class IsLaneletTurnDirectionCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  IsLaneletTurnDirectionCondition(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("lanelet"),
      BT::InputPort<std::string>("expected"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "lanelet");
    if (!ports::require(lanelet, "lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto expected = ports::tryGet<std::string>(*this, "expected");
    if (!ports::require(expected, "expected", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (lanelet->turn_direction == *expected) {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(
      logger(), "turn_direction_mismatch lanelet_id=" << lanelet->id
                << " actual=" << lanelet->turn_direction
                << " expected=" << *expected);
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__IS_LANELET_TURN_DIRECTION_CONDITION_HPP_
