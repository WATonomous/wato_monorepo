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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/intersection.hpp"
#include "behaviour/utils/lanelet.hpp"
#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/types.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/regulatory_element.hpp"

namespace behaviour
{
/**
   * @class GetIntersectionContextAction
   * @brief SyncActionNode to select and latch upcoming traffic control context.
   *
   * Logic:
   * - If an active traffic control element is already latched (car is currently dealing with it), return it and `SUCCESS`.
   *   Full reset of subtype-specific state is handled later by `ResetIntersectionContext`.
   * - Otherwise, inspect `search_lanelets` in order to find the next relevant control element.
   * - Select control priority as: `traffic_light` > `stop_sign` > `yield`.
   * - Write selected control element and lanelet metadata to output ports.
   * - Return `SUCCESS` when a valid context is found and published.
   * - Return `FAILURE` only when required context is missing.
   *
   * Assumptions:
   * - Regulatory element raw subtype and attributes can be normalized into BT control types.
   * - Priority order is fixed to traffic light, then stop sign, then yield.
 */
class GetIntersectionContextAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetIntersectionContextAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<std::vector<lanelet_msgs::msg::Lanelet>>("search_lanelets"),
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("in_active_traffic_control_element"),
      BT::InputPort<int64_t>("in_active_traffic_control_lanelet_id"),
      BT::OutputPort<int64_t>("out_active_traffic_control_lanelet_id"),
      BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("out_active_traffic_control_element"),
      BT::OutputPort<int64_t>("out_active_traffic_control_element_id"),
      BT::OutputPort<double>("out_distance_to_intersection_m"),
      BT::OutputPort<bool>("out_passing_active_traffic_control_element"),
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

    auto search_lanelets = ports::tryGet<std::vector<lanelet_msgs::msg::Lanelet>>(*this, "search_lanelets");
    if (!ports::require(search_lanelets, "search_lanelets", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    // If an active element is already latched, preserve it here so the tree can
    // run the full reset path in ResetIntersectionContext. Clearing only the
    // active element here would leave subtype-specific blackboard state stale.
    auto active_traffic_control_element =
      ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "in_active_traffic_control_element");

    if (active_traffic_control_element) {
      auto active_lanelet_id = ports::tryGet<int64_t>(*this, "in_active_traffic_control_lanelet_id");
      publish_active_context(utils::intersection::makeActiveTrafficControlContext(
        *lane_ctx, *search_lanelets, active_lanelet_id.value_or(ports::null_id), active_traffic_control_element));
      return BT::NodeStatus::SUCCESS;
    }

    auto next_active_context = utils::intersection::findNextActiveTrafficControlContext(*lane_ctx, *search_lanelets);
    if (next_active_context) {
      const auto elem_type = utils::lanelet::getTrafficControlElementType(*next_active_context->element);
      RCLCPP_DEBUG_STREAM(
        logger(),
        "latched_control lanelet_id=" << next_active_context->lanelet_id
                                      << " raw_subtype=" << next_active_context->element->subtype
                                      << " type=" << (elem_type ? types::toString(*elem_type) : "unknown"));
      publish_active_context(*next_active_context);
      return BT::NodeStatus::SUCCESS;
    }

    clear_active_outputs();

    RCLCPP_DEBUG_STREAM(logger(), "no_control_element search_count=" << search_lanelets->size());
    return BT::NodeStatus::SUCCESS;
  }

private:
  void publish_active_context(const utils::intersection::ActiveTrafficControlContext & context)
  {
    setOutput("out_active_traffic_control_lanelet_id", context.lanelet_id);
    setOutput("out_active_traffic_control_element", context.element);
    setOutput("out_active_traffic_control_element_id", context.element_id);
    setOutput("out_distance_to_intersection_m", context.distance_to_intersection_m);
    setOutput("out_passing_active_traffic_control_element", context.passing_active_traffic_control_element);
  }

  /**
     * @brief Clear all active traffic control element outputs (full latch reset).
     */
  void clear_active_outputs()
  {
    lanelet_msgs::msg::RegulatoryElement::SharedPtr cleared = nullptr;
    setOutput("out_active_traffic_control_element", cleared);
    setOutput("out_active_traffic_control_lanelet_id", static_cast<int64_t>(0));
    setOutput("out_active_traffic_control_element_id", static_cast<int64_t>(0));
    setOutput("out_passing_active_traffic_control_element", false);
    setOutput("out_distance_to_intersection_m", -1.0);
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_CONTEXT_ACTION_HPP_
