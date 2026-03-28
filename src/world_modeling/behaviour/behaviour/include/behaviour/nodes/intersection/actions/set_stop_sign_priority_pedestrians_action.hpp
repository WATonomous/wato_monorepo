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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_PEDESTRIANS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_PEDESTRIANS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
 * @class SetStopSignPriorityPedestriansAction
 * @brief SyncActionNode to latch stop-sign priority pedestrian IDs.
 */
class SetStopSignPriorityPedestriansAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetStopSignPriorityPedestriansAction(
    const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("current_pedestrian_ids"),
      BT::InputPort<std::vector<std::string>>("priority_pedestrian_ids"),
      BT::InputPort<bool>("priority_latched"),

      BT::OutputPort<std::vector<std::string>>("out_priority_pedestrian_ids"),
      BT::OutputPort<bool>("out_priority_latched"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    const bool latched = ports::tryGet<bool>(*this, "priority_latched").value_or(false);
    if (latched) {
      auto existing = ports::tryGet<std::vector<std::string>>(*this, "priority_pedestrian_ids");
      if (!ports::require(existing, "priority_pedestrian_ids", missing_input_callback)) {
        return BT::NodeStatus::FAILURE;
      }
      setOutput("out_priority_pedestrian_ids", *existing);
      setOutput("out_priority_latched", true);
      return BT::NodeStatus::SUCCESS;
    }

    auto current = ports::tryGet<std::vector<std::string>>(*this, "current_pedestrian_ids");
    if (!ports::require(current, "current_pedestrian_ids", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("out_priority_pedestrian_ids", *current);
    setOutput("out_priority_latched", true);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_PEDESTRIANS_ACTION_HPP_
