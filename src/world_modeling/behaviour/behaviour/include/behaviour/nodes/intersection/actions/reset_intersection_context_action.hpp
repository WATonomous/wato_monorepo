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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__RESET_INTERSECTION_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__RESET_INTERSECTION_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{

/**
 * @class ResetIntersectionContextAction
 * @brief SyncActionNode that clears the active traffic control element and
 *        resets the wall_id for the matching element type.
 */
class ResetIntersectionContextAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  ResetIntersectionContextAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>(
        "active_traffic_control_element", "Current element (read type before clearing)"),
      BT::InputPort<int32_t>("stop_sign_wall_id", "Current stop sign wall ID"),
      BT::InputPort<int32_t>("traffic_light_wall_id", "Current traffic light wall ID"),
      BT::InputPort<int32_t>("yield_wall_id", "Current yield wall ID"),
      BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>(
        "out_active_traffic_control_element", "Cleared element"),
      BT::OutputPort<int64_t>("out_active_traffic_control_lanelet_id", "Cleared lanelet ID"),
      BT::OutputPort<int64_t>("out_active_traffic_control_element_id", "Cleared element ID"),
      BT::OutputPort<int32_t>("out_stop_sign_wall_id", "Reset stop sign wall ID"),
      BT::OutputPort<std::vector<std::string>>("out_stop_sign_priority_car_ids", "Reset stop sign priority car IDs"),
      BT::OutputPort<std::vector<std::string>>(
        "out_stop_sign_priority_pedestrian_ids", "Reset stop sign priority pedestrian IDs"),
      BT::OutputPort<bool>("out_stop_sign_priority_latched", "Reset stop sign priority latch"),
      BT::OutputPort<bool>(
        "out_stop_sign_priority_pedestrian_latched", "Reset stop sign priority pedestrian latch"),
      BT::OutputPort<bool>("out_stop_sign_ego_priority", "Reset stop sign ego priority latch"),
      BT::OutputPort<int32_t>("out_traffic_light_wall_id", "Reset traffic light wall ID"),
      BT::OutputPort<bool>(
        "out_traffic_light_right_turn_release_latched", "Reset traffic light right-turn release latch"),
      BT::OutputPort<int32_t>("out_yield_wall_id", "Reset yield wall ID"),
      BT::OutputPort<bool>("out_yield_ego_priority", "Reset yield ego priority latch"),
      BT::OutputPort<int32_t>("out_active_wall_id", "Wall ID that needs despawning"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "active_traffic_control_element");

    int32_t active_wall_id = 0;

    // clear the wall corresponding to the regulatory element subtype, and record the active wall ID for despawning
    if (reg_elem) {
      const auto normalized_type = utils::lanelet::getTrafficControlElementType(*reg_elem);
      if (normalized_type && *normalized_type == types::TrafficControlElementType::STOP_SIGN) {
        auto wid = ports::tryGet<int32_t>(*this, "stop_sign_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_stop_sign_wall_id", ports::null_id);
        setOutput("out_stop_sign_priority_car_ids", std::vector<std::string>{});
        setOutput("out_stop_sign_priority_pedestrian_ids", std::vector<std::string>{});
        setOutput("out_stop_sign_priority_latched", false);
        setOutput("out_stop_sign_priority_pedestrian_latched", false);
        setOutput("out_stop_sign_ego_priority", false);
      } else if (normalized_type && *normalized_type == types::TrafficControlElementType::TRAFFIC_LIGHT) {
        auto wid = ports::tryGet<int32_t>(*this, "traffic_light_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_traffic_light_wall_id", ports::null_id);
      } else if (normalized_type && *normalized_type == types::TrafficControlElementType::YIELD) {
        auto wid = ports::tryGet<int32_t>(*this, "yield_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_yield_wall_id", ports::null_id);
        setOutput("out_yield_ego_priority", false);
      }
      RCLCPP_DEBUG_STREAM(
        logger(),
        "raw_subtype=" << reg_elem->subtype
                       << " type=" << (normalized_type ? types::toString(*normalized_type) : "unknown")
                       << " wall_id=" << active_wall_id);
    } else {
      RCLCPP_DEBUG_STREAM(logger(), "no_active_element=true");
    }

    setOutput("out_traffic_light_right_turn_release_latched", false);

    // wall id to despawn
    setOutput("out_active_wall_id", active_wall_id);

    lanelet_msgs::msg::RegulatoryElement::SharedPtr cleared = nullptr;
    setOutput("out_active_traffic_control_element", cleared);
    setOutput("out_active_traffic_control_lanelet_id", ports::null_id);
    setOutput("out_active_traffic_control_element_id", ports::null_id);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__RESET_INTERSECTION_CONTEXT_ACTION_HPP_
