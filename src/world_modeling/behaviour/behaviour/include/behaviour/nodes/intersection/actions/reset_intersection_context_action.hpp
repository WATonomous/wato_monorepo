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

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{

/**
 * @class ResetIntersectionContextAction
 * @brief SyncActionNode that clears the active traffic control element and
 *        resets the wall_id for the matching element type.
 */
class ResetIntersectionContextAction : public BT::SyncActionNode
{
public:
  ResetIntersectionContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
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
      BT::OutputPort<int32_t>("out_traffic_light_wall_id", "Reset traffic light wall ID"),
      BT::OutputPort<int32_t>("out_yield_wall_id", "Reset yield wall ID"),
      BT::OutputPort<int32_t>("out_active_wall_id", "Wall ID that needs despawning"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto reg_elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(
      *this, "active_traffic_control_element");

    int32_t active_wall_id = 0;

    if (reg_elem) {
      const auto & subtype = reg_elem->subtype;
      if (subtype == "stop_sign") {
        auto wid = ports::tryGet<int32_t>(*this, "stop_sign_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_stop_sign_wall_id", static_cast<int32_t>(0));
      } else if (subtype == "traffic_light") {
        auto wid = ports::tryGet<int32_t>(*this, "traffic_light_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_traffic_light_wall_id", static_cast<int32_t>(0));
      } else if (subtype == "yield") {
        auto wid = ports::tryGet<int32_t>(*this, "yield_wall_id");
        if (wid) active_wall_id = *wid;
        setOutput("out_yield_wall_id", static_cast<int32_t>(0));
      }
      std::cout << "[ResetIntersectionContext]: Clearing element (subtype=" << subtype
                << ", wall_id=" << active_wall_id << ")" << std::endl;
    } else {
      std::cout << "[ResetIntersectionContext]: No active element to clear" << std::endl;
    }

    setOutput("out_active_wall_id", active_wall_id);

    lanelet_msgs::msg::RegulatoryElement::SharedPtr cleared = nullptr;
    setOutput("out_active_traffic_control_element", cleared);
    setOutput("out_active_traffic_control_lanelet_id", static_cast<int64_t>(0));
    setOutput("out_active_traffic_control_element_id", static_cast<int64_t>(0));

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__RESET_INTERSECTION_CONTEXT_ACTION_HPP_
