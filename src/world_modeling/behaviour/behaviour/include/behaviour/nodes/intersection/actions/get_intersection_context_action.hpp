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
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
   * @class GetIntersectionContextAction
   * @brief SyncActionNode to select and latch upcoming traffic control context.
   *
   * Logic:
   * - If an active traffic control element is already latched (car is currently dealing with it), return it and `SUCCESS`.
   * - Otherwise, inspect the route ahead (globla route) of ego up to `lookahead_threshold_m` and find
   *   the first relevant regulatory element.
   * - Select control priority as: `traffic_light` > `stop_sign` > `yield`.
   * - Write selected control element and lanelet metadata to output ports.
   * - Return `SUCCESS` when a valid context is found and published.
   * - Return `FAILURE` when required context is missing or no valid control element can be derived.
   *
   * Assumptions:
   * - `route_index_map` is consistent with `route`.
   * - Regulatory element subtype strings match expected classifier values.
   * - Priority order is fixed to traffic light, then stop sign, then yield.
   */
class GetIntersectionContextAction : public BT::SyncActionNode
{
public:
  GetIntersectionContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr>("route"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("route_index_map"),
      BT::InputPort<double>("lookahead_threshold_m"),
      BT::InputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("in_active_traffic_control_element"),
      BT::OutputPort<int64_t>("out_active_traffic_control_lanelet_id"),
      BT::OutputPort<lanelet_msgs::msg::RegulatoryElement::SharedPtr>("out_active_traffic_control_element"),
      BT::OutputPort<int64_t>("out_active_traffic_control_element_id"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto active_traffic_control_element =
      ports::tryGetPtr<lanelet_msgs::msg::RegulatoryElement>(*this, "in_active_traffic_control_element");
    if (active_traffic_control_element) {
      setOutput("out_active_traffic_control_element", active_traffic_control_element);
      return BT::NodeStatus::SUCCESS;
    }

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    auto route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "route");
    auto route_index_map = ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "route_index_map");
    auto lookahead_threshold_m = ports::tryGet<double>(*this, "lookahead_threshold_m").value_or(40.0);

    if (!lane_ctx) {
      std::cerr << "[GetIntersectionContext] Missing lane_ctx." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!route) {
      std::cerr << "[GetIntersectionContext] Missing route." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!route_index_map) {
      std::cerr << "[GetIntersectionContext] Missing route_index_map." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // ------------- using lookahead distance -------------
    // check the current lanelet first
    const auto lanelet_id = lane_ctx->current_lanelet.id;
    const auto route_it = route_index_map->find(lanelet_id);
    if (route_it != route_index_map->end()) {
      const auto & lanelet = route->lanelets[route_it->second];
      if (auto elem = classify_lanelet_traffic_control_element(lanelet)) {
        setOutput("out_active_traffic_control_lanelet_id", lanelet_id);
        setOutput("out_active_traffic_control_element", elem);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // now check the upcoming lanelets within the lookahead distance
    const std::size_t m =
      std::min(lane_ctx->upcoming_lanelet_ids.size(), lane_ctx->upcoming_lanelet_distances_m.size());

    for (std::size_t i = 0; i < m; ++i) {
      const double dist = lane_ctx->upcoming_lanelet_distances_m[i];
      if (dist < 0.0) continue;  // if your msg ever uses -1
      if (dist > lookahead_threshold_m) break;  // distances are increasing in order

      const int64_t lanelet_id = lane_ctx->upcoming_lanelet_ids[i];

      const auto route_it = route_index_map->find(lanelet_id);
      if (route_it == route_index_map->end()) continue;

      const auto & lanelet = route->lanelets[route_it->second];

      if (auto elem = classify_lanelet_traffic_control_element(lanelet)) {
        setOutput("out_active_traffic_control_lanelet_id", lanelet_id);
        setOutput("out_active_traffic_control_element", elem);
        setOutput("out_active_traffic_control_element_id", elem->id);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // don't set the outputs if nothing found (no active traffic control element)
    return BT::NodeStatus::SUCCESS;
    // ------------- using lookahead distance -------------
  }

private:
  inline static constexpr std::string_view traffic_light_subtype_ = "traffic_light";
  inline static constexpr std::string_view stop_sign_subtype_ = "stop_sign";
  inline static constexpr std::string_view yield_subtype_ = "yield";

  static int priority(std::string_view subtype)
  {
    if (subtype == traffic_light_subtype_) return 0;
    if (subtype == stop_sign_subtype_) return 1;
    if (subtype == yield_subtype_) return 2;
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
      const int p = priority(reg_elem.subtype);
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
