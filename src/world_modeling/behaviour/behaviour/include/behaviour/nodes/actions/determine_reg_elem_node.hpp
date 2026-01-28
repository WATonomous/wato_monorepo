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

#ifndef BEHAVIOUR__NODES__ACTIONS__DETERMINE_REG_ELEM_NODE_HPP_
#define BEHAVIOUR__NODES__ACTIONS__DETERMINE_REG_ELEM_NODE_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class DetermineRegElemAction
   * @brief BT node to detect upcoming regulatory elements (intersections, traffic lights, etc.)
   * based on the current lane context and upcoming path.
   *
   * TODO(wato): This node needs significant improvement. Deducing reg elem types and IDs
   * from lanelet flags is fragile. We need better world_model services that can provide
   * a structured "Regulatory Context" for the current route.
   */
class DetermineRegElemAction : public BT::SyncActionNode
{
public:
  DetermineRegElemAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::LaneletPtr>("target_lanelet"),

      // Output Ports
      BT::OutputPort<std::string>(
        "reg_elem_type", "Type of the upcoming element (intersection, traffic_light, stop_line, yield_sign, none)"),
    };
  }

  /**
     * @brief Continuously checks the successor lanelet in the current route for regulatory flags.
     */
  BT::NodeStatus tick() override
  {
    types::LaneletPtr target_lanelet;
    try {
      target_lanelet = ports::getPtr<types::Lanelet>(*this, "target_lanelet");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (target_lanelet->is_intersection) {
      setOutput("reg_elem_type", std::string("intersection"));
    } else if (target_lanelet->has_traffic_light) {
      setOutput("reg_elem_type", std::string("traffic_light"));
    } else if (target_lanelet->has_stop_line) {
      setOutput("reg_elem_type", std::string("stop_line"));
    } else if (target_lanelet->has_yield_sign) {
      setOutput("reg_elem_type", std::string("yield_sign"));
    } else {
      setOutput("reg_elem_type", std::string("none"));
    }

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__DETERMINE_REG_ELEM_NODE_HPP_
