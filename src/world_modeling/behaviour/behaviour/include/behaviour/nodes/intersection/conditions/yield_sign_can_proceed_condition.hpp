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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__YIELD_SIGN_CAN_PROCEED_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__YIELD_SIGN_CAN_PROCEED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "behaviour/utils/object_utils.hpp"
#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
 * @class YieldSignCanProceedCondition
 * @brief ConditionNode to check whether yield lanelets are clear.
 *
 * Logic:
 * - Read the active control element and current world objects.
 * - Filter objects to cars on lanelets listed in `yield_lanelet_ids`.
 * - Return `SUCCESS` only when all yield lanelets are clear.
 * - Return `FAILURE` as soon as any yield lanelet contains a car.
 * - Return `FAILURE` when required state is missing (fail-safe behavior).
 *
 * Assumptions:
 * - `yield_lanelet_ids` correctly encodes conflicting traffic lanes.
 * - If there are no more cars in the yield lanelets, then it's safe to proceed, car might still pose a risk in we do allow the car to go
 * - World object lanelet association is up to date and reliable.
 */
class YieldSignCanProceedCondition : public BT::ConditionNode
{
public:
  YieldSignCanProceedCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("active_traffic_control_element"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<int>("hypothesis_index"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[YieldSignCanProceed]: Missing " << port_name << " input" << std::endl;
    };

    auto elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "active_traffic_control_element");
    if (!ports::require(elem, "active_traffic_control_element", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto hypothesis_index = ports::tryGet<int>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (elem->yield_lanelet_ids.empty()) {
      std::cout << "[YieldSignCanProceed]: yield_lanelet_ids empty (fail-safe)" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (*hypothesis_index < 0) {
      std::cout << "[YieldSignCanProceed]: invalid hypothesis_index" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    for (const auto lanelet_id : elem->yield_lanelet_ids) {
      const auto cars = object_utils::getCarsByLanelet(*objects, *hypothesis_index, lanelet_id);
      if (!cars.empty()) {
        std::cout << "[YieldSignCanProceed]: Blocked (car in yield lanelet " << lanelet_id << ")" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }

    std::cout << "[YieldSignCanProceed]: Clear (no cars in yield lanelets)" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__YIELD_SIGN_CAN_PROCEED_CONDITION_HPP_
