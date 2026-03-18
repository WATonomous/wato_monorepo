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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PEDESTRIANS_CLEAR_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PEDESTRIANS_CLEAR_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/logged_bt_node.hpp"

#include <iostream>
#include <string>
#include <vector>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class StopSignPedestriansClearCondition
 * @brief Returns SUCCESS when no pedestrians are on the yield lanelets, FAILURE otherwise.
 */
class StopSignPedestriansClearCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  StopSignPedestriansClearCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("pedestrian_ids"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto pedestrian_ids = ports::tryGet<std::vector<std::string>>(*this, "pedestrian_ids");
    if (!pedestrian_ids || pedestrian_ids->empty()) {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(logger(), "Blocked (" << pedestrian_ids->size()
              << " pedestrian(s) crossing)" );
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PEDESTRIANS_CLEAR_CONDITION_HPP_
