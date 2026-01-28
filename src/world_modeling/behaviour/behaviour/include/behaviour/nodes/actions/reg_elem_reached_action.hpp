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

#ifndef BEHAVIOUR__NODES__ACTIONS__REG_ELEM_REACHED_ACTION_HPP_
#define BEHAVIOUR__NODES__ACTIONS__REG_ELEM_REACHED_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{
/**
   * @class RegElemReachedAction
   * @brief BT action node to check if the first upcoming regulatory element is within a threshold distance.
   * If reached, it outputs the selected element for downstream guards.
   *
   * Inputs:
   *   - reg_elems: Array of RegulatoryElement structs found in path
   *   - threshold_m: Distance threshold in meters
   *
   * Outputs:
   *   - selected_reg_elem: The specific element that passed the check
   */
class RegElemReachedAction : public BT::SyncActionNode
{
public:
  RegElemReachedAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<types::RegulatoryElementArrayPtr>("reg_elems", "Array of regulatory elements"),
      BT::InputPort<double>("threshold_m", 2.0, "Distance threshold in meters"),
      BT::OutputPort<types::RegulatoryElementPtr>("selected_reg_elem", "The element that was reached")};
  }

  BT::NodeStatus tick() override
  {
    types::RegulatoryElementArrayPtr reg_elems;
    double threshold_m;

    try {
      reg_elems = ports::getPtr<types::RegulatoryElementArray>(*this, "reg_elems");
      threshold_m = ports::get<double>(*this, "threshold_m");
    } catch (const BT::RuntimeError & e) {
      return BT::NodeStatus::FAILURE;
    }

    if (reg_elems->empty()) {
      return BT::NodeStatus::FAILURE;
    }

    // Check the first regulatory element's distance
    if (reg_elems->front().distance <= threshold_m) {
      setOutput("selected_reg_elem", std::make_shared<types::RegulatoryElement>(reg_elems->front()));
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__ACTIONS__REG_ELEM_REACHED_ACTION_HPP_
