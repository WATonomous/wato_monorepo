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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PRIORITY_ACTORS_LATCHED_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PRIORITY_ACTORS_LATCHED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class StopSignPriorityActorsLatchedCondition
 * @brief Returns SUCCESS once both stop-sign actor latches have been initialized.
 */
class StopSignPriorityActorsLatchedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  StopSignPriorityActorsLatchedCondition(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("priority_cars_latched"),
      BT::InputPort<bool>("priority_pedestrians_latched"),
    };
  }

  BT::NodeStatus tick() override
  {
    const bool cars_latched = ports::tryGet<bool>(*this, "priority_cars_latched").value_or(false);
    const bool pedestrians_latched = ports::tryGet<bool>(*this, "priority_pedestrians_latched").value_or(false);
    return (cars_latched && pedestrians_latched) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__STOP_SIGN_PRIORITY_ACTORS_LATCHED_CONDITION_HPP_
