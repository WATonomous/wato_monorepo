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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class GetRouteContextAction
 * @brief SyncActionNode to derive route transition and next lanelet from current context.
 *
 * Logic:
 * - Resolve ego lanelet index from `route_index_map`.
 * - If ego is not on the route, return `FAILURE`.
 * - If ego is at the last route lanelet, emit `SUCCESSOR` and current lanelet, then return `SUCCESS`.
 * - Otherwise, emit transition at current index and the next route lanelet, then return `SUCCESS`.
 * - Return `FAILURE` when required context is missing.
 *
 * Assumptions:
 * - `route_index_map` and `route` refer to the same route ordering.
 * - Transition list is aligned with lanelet ordering by index.
 */
class GetRouteContextAction : public BT::SyncActionNode
{
public:
  GetRouteContextAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<lanelet_msgs::srv::GetShortestRoute::Response>>("route"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("route_index_map"),
      BT::InputPort<std::shared_ptr<lanelet_msgs::msg::CurrentLaneContext>>("lane_ctx"),
      BT::OutputPort<types::LaneTransition>("out_lane_transition"),
      BT::OutputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("out_next_lanelet")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GetRouteContextAction]: Missing " << port_name << " input" << std::endl;
    };

    auto route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "route");
    if (!ports::require(route, "route", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto map = ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "route_index_map");
    if (!ports::require(map, "route_index_map", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto current_lane_context = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(current_lane_context, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const int64_t current_id = current_lane_context->current_lanelet.id;
    auto current_it = map->find(current_id);
    if (current_it == map->end()) {
      // Current lanelet not found on route
      std::cout << "[GetRouteContext]: Current lanelet ID " << current_id << " not found on route" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    const std::size_t current_idx = current_it->second;
    if (current_idx >= route->lanelets.size() - 1) {
      // At last lanelet, no further transitions
      setOutput("out_lane_transition", types::LaneTransition::SUCCESSOR);

      auto current_lanelet_ptr = std::make_shared<lanelet_msgs::msg::Lanelet>(current_lane_context->current_lanelet);
      setOutput("out_next_lanelet", current_lanelet_ptr);

      std::cout << "[GetRouteContext]: Result=SUCCESS (end of route, default SUCCESSOR)" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    // Extract transition and next lanelet on route from the current lanelet (target_lanelet)
    const uint8_t transition = route->transitions[current_idx];
    const types::LaneTransition transition_enum = static_cast<types::LaneTransition>(transition);

    const lanelet_msgs::msg::Lanelet::SharedPtr next_lanelet =
      std::make_shared<lanelet_msgs::msg::Lanelet>(route->lanelets[current_idx + 1]);

    setOutput("out_lane_transition", transition_enum);
    setOutput("out_next_lanelet", next_lanelet);
    std::cout << "[GetRouteContext]: Result=SUCCESS (transition set, next_lanelet_id=" << next_lanelet->id << ")"
              << std::endl;

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_ROUTE_CONTEXT_ACTION_HPP_
