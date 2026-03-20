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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
   * @class GetTrafficLightStateAction
   * @brief SyncActionNode to read traffic light state from world-object snapshots.
   */
class GetTrafficLightStateAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetTrafficLightStateAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("traffic_light"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<std::size_t>("state_hypothesis_index"),
      BT::OutputPort<std::string>("out_traffic_light_state"),
      BT::OutputPort<std::string>("error_message"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "traffic_light");
    if (!ports::require(reg_elem, "traffic_light", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto state_hypothesis_index = ports::tryGet<std::size_t>(*this, "state_hypothesis_index");
    if (!ports::require(state_hypothesis_index, "state_hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t reg_elem_id = reg_elem->id;
    const auto state = world_objects::getTrafficLightState(reg_elem_id, *state_hypothesis_index, *objects);
    if (!state) {
      setOutput("error_message", "traffic_light_state_not_found");
      RCLCPP_DEBUG_STREAM(logger(), "failed to get state for reg_elem_id=" << reg_elem_id );
      return BT::NodeStatus::FAILURE;
    }

    setOutput("out_traffic_light_state", *state);
    RCLCPP_DEBUG_STREAM(logger(), "state=" << *state << " reg_elem_id=" << reg_elem_id );

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_TRAFFIC_LIGHT_STATE_ACTION_HPP_
