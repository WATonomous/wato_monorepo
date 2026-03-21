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

#include "behaviour/nodes/bt_logger_base.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behaviour/utils/utils.hpp"
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
   * - Otherwise, inspect `search_lanelets` in order to find the next relevant control element.
   * - Select control priority as: `traffic_light` > `stop_sign` > `yield`.
   * - Write selected control element and lanelet metadata to output ports.
   * - Return `SUCCESS` when a valid context is found and published.
   * - Return `FAILURE` only when required context is missing.
   *
   * Assumptions:
   * - `search_lanelet_index_map` is consistent with `search_lanelets`.
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
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("search_lanelet_index_map"),
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("in_active_traffic_control_element"),
      BT::InputPort<int64_t>("in_active_traffic_control_lanelet_id"),
      BT::OutputPort<int64_t>("out_active_traffic_control_lanelet_id"),
      BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("out_active_traffic_control_element"),
      BT::OutputPort<int64_t>("out_active_traffic_control_element_id"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input" );
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto search_lanelets = ports::tryGet<std::vector<lanelet_msgs::msg::Lanelet>>(*this, "search_lanelets");
    if (!ports::require(search_lanelets, "search_lanelets", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto search_lanelet_index_map =
      ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "search_lanelet_index_map");
    if (!ports::require(search_lanelet_index_map, "search_lanelet_index_map", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    // if active element input is provided, it means we are currently latched to an element and should validate it before deciding whether to keep or drop the latch
    auto active_traffic_control_element =
      ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "in_active_traffic_control_element");

    if (active_traffic_control_element) {
      auto active_lanelet_id = ports::tryGet<int64_t>(*this, "in_active_traffic_control_lanelet_id");

      if (active_lanelet_id && is_latch_stale(*active_lanelet_id, *lane_ctx)) {
        // ego has left the intersection zone and the active lanelet is no longer ahead
        RCLCPP_DEBUG_STREAM(logger(), "Stale latch on lanelet " << *active_lanelet_id
                  << ", clearing" );
        clear_active_outputs();
        // fall through to rescan for a new element below
      } else {
        // if valid, keep the latch
        if (active_lanelet_id) {
          setOutput("out_active_traffic_control_lanelet_id", *active_lanelet_id);
        }
        setOutput("out_active_traffic_control_element_id", active_traffic_control_element->id);
        setOutput("out_active_traffic_control_element", active_traffic_control_element);
        return BT::NodeStatus::SUCCESS;
      }
    }

    for (const auto & lanelet : *search_lanelets) {
      if (auto elem = classify_lanelet_traffic_control_element(lanelet)) {
        const auto elem_type = utils::lanelet::getTrafficControlElementType(*elem);
        RCLCPP_DEBUG_STREAM(logger(), "Latched lanelet=" << lanelet.id
                  << " raw_subtype=" << elem->subtype
                  << " normalized_type=" << (elem_type ? types::toString(*elem_type) : "unknown") );
        setOutput("out_active_traffic_control_lanelet_id", lanelet.id);
        setOutput("out_active_traffic_control_element", elem);
        setOutput("out_active_traffic_control_element_id", elem->id);
        return BT::NodeStatus::SUCCESS;
      }
    }

    RCLCPP_DEBUG_STREAM(logger(), "No control element found"
              << " (search_count=" << search_lanelets->size() << ")" );
    return BT::NodeStatus::SUCCESS;
  }

private:
  /**
     * @brief Determine whether the latched active element is stale using lane context.
     *
     * The latch is stale when ALL of the following are true:
     *   1. Ego is NOT on the active element's lanelet.
     *   2. Ego's current lanelet is NOT an intersection lanelet.
     *   3. The active element's lanelet does NOT appear in the upcoming lanelet list.
     *
     * This avoids relying on lanelet-index counts (which break for long lanelets)
     * and instead uses the physical lane context: if the car has left the
     * intersection zone and the active lanelet is no longer ahead, the latch is
     * stale and should be cleared.
     */
  static bool is_latch_stale(int64_t active_lanelet_id, const lanelet_msgs::msg::CurrentLaneContext & lane_ctx)
  {
    const int64_t ego_lanelet_id = lane_ctx.current_lanelet.id;

    // still on the active lanelet
    if (ego_lanelet_id == active_lanelet_id) {
      return false;
    }

    // still inside an intersection lanelet
    if (lane_ctx.current_lanelet.is_intersection) {
      return false;
    }

    // 3. The active lanelet is still ahead in the upcoming path
    for (const auto & upcoming_id : lane_ctx.upcoming_lanelet_ids) {
      if (upcoming_id == active_lanelet_id) {
        return false;
      }
    }

    // None of the keep-alive conditions hold — the latch is stale
    return true;
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
  }

  static int priority(types::TrafficControlElementType type)
  {
    if (type == types::TrafficControlElementType::TRAFFIC_LIGHT) return 0;
    if (type == types::TrafficControlElementType::STOP_SIGN) return 1;
    if (type == types::TrafficControlElementType::YIELD) return 2;
    return 999;
  }

  // find the best candidate that represents the primary regulatory element on the lanelet
  // since lanelets can have multiple regulatory elements attached
  static lanelet_msgs::msg::RegulatoryElement::SharedPtr classify_lanelet_traffic_control_element(
    const lanelet_msgs::msg::Lanelet & lanelet)
  {
    lanelet_msgs::msg::RegulatoryElement::SharedPtr primary_reg_elem = nullptr;

    // lower is higher priority
    int best_prio = 999;

    for (const auto & reg_elem : lanelet.regulatory_elements) {
      const auto elem_type = utils::lanelet::getTrafficControlElementType(reg_elem);
      if (!elem_type) {
        continue;
      }

      const int p = priority(*elem_type);
      if (p < best_prio) {
        best_prio = p;
        primary_reg_elem = std::make_shared<lanelet_msgs::msg::RegulatoryElement>(reg_elem);

        // early exit if we found the top priority
        if (best_prio == 0) {
          break;
        }
      }
    }

    // nullptr if none found
    return primary_reg_elem;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_INTERSECTION_CONTEXT_ACTION_HPP_
