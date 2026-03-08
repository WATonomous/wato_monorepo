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

#ifndef BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_PASSED_CONDITION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_PASSED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <cstdint>
#include <iostream>
#include <string>

#include "behaviour/utils/intersection.hpp"
#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour
{
/**
   * @class ActiveTrafficControlElementPassedCondition
   * @brief ConditionNode to check whether ego has passed the active traffic-control lanelet.
   *
   * Logic (purely lane-context based, no lanelet-index counting):
   *   The element is considered "passed" when ALL of the following hold:
   *     1. Ego is NOT on the active element's lanelet.
   *     2. Ego's current lanelet is NOT an intersection lanelet.
   *     3. The active element's lanelet does NOT appear in `upcoming_lanelet_ids`.
   *
   * This avoids relying on lanelet-index counts (which break for long lanelets)
   * and instead uses the physical lane context.
   *
   * Returns:
   *   - `SUCCESS` when ego has passed the intersection zone.
   *   - `FAILURE` when ego is still in or approaching the intersection (fail-safe).
   */
class ActiveTrafficControlElementPassedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  ActiveTrafficControlElementPassedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<int64_t>("active_traffic_control_lanelet_id"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "missing_input port=" << port_name);
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto active_lanelet_id = ports::tryGet<int64_t>(*this, "active_traffic_control_lanelet_id");
    if (!ports::require(active_lanelet_id, "active_traffic_control_lanelet_id", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t active_id = *active_lanelet_id;
    if (!utils::intersection::hasPassedActiveTrafficControlElement(active_id, *lane_ctx)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_PASSED_CONDITION_HPP_
