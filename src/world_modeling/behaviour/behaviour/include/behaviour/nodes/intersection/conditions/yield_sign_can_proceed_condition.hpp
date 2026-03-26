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

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{
/**
   * @class YieldSignCanProceedCondition
 * @brief ConditionNode to check whether ego may proceed through a right-of-way relation.
   *
   * Logic:
   * - Read the active control element and current world objects.
   * - Determine ego's role in the active right-of-way relation.
   * - If ego has right of way, return `SUCCESS` immediately.
   * - If ego is yielding, filter objects to cars on lanelets listed in `right_of_way_lanelet_ids`.
   * - Return `SUCCESS` only when all priority lanelets are clear.
   * - Return `FAILURE` as soon as any priority lanelet contains a car.
   * - Return `FAILURE` when required state is missing (fail-safe behavior).
   *
   * Assumptions:
   * - `right_of_way_lanelet_ids` encodes conflicting traffic lanes when ego is yielding.
   * - World object lanelet association is up to date and reliable.
   */
class YieldSignCanProceedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  YieldSignCanProceedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("active_traffic_control_element"),
      BT::InputPort<int64_t>("active_traffic_control_lanelet_id"),
      BT::InputPort<std::vector<world_model_msgs::msg::WorldObject>>("objects"),
      BT::InputPort<std::size_t>("hypothesis_index"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto elem = ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "active_traffic_control_element");
    if (!ports::require(elem, "active_traffic_control_element", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto objects = ports::tryGet<std::vector<world_model_msgs::msg::WorldObject>>(*this, "objects");
    if (!ports::require(objects, "objects", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto active_lanelet_id = ports::tryGet<int64_t>(*this, "active_traffic_control_lanelet_id");
    if (!ports::require(active_lanelet_id, "active_traffic_control_lanelet_id", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto hypothesis_index = ports::tryGet<std::size_t>(*this, "hypothesis_index");
    if (!ports::require(hypothesis_index, "hypothesis_index", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto role = utils::lanelet::getRightOfWayRole(*elem, *active_lanelet_id);
    if (role == utils::lanelet::RightOfWayRole::RIGHT_OF_WAY) {
      return BT::NodeStatus::SUCCESS;
    }

    if (role != utils::lanelet::RightOfWayRole::YIELD) {
      RCLCPP_DEBUG_STREAM(logger(), "yield_fail_safe reason=role_not_applicable lanelet_id=" << *active_lanelet_id);
      return BT::NodeStatus::FAILURE;
    }

    if (elem->right_of_way_lanelet_ids.empty()) {
      RCLCPP_DEBUG_STREAM(
        logger(), "yield_fail_safe reason=missing_priority_lanelets lanelet_id=" << *active_lanelet_id);
      return BT::NodeStatus::FAILURE;
    }

    for (const auto lanelet_id : elem->right_of_way_lanelet_ids) {
      const auto cars = utils::world_objects::getCarsByLanelet(*objects, *hypothesis_index, lanelet_id);
      if (!cars.empty()) {
        RCLCPP_DEBUG_STREAM(logger(), "yield_blocked priority_lanelet_id=" << lanelet_id);
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__YIELD_SIGN_CAN_PROCEED_CONDITION_HPP_
