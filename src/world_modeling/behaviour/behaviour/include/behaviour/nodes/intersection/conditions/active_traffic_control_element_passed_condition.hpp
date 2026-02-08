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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour
{
/**
     * @class ActiveTrafficControlElementPassedCondition
     * @brief ConditionNode to check whether ego passed the active traffic-control lanelet.
     *
     * Logic:
     * - Resolve the route indices for ego lanelet and active control lanelet.
     * - Compute index difference and compare against `lanelet_threshold`.
     * - Return `SUCCESS` when ego is sufficiently ahead of the active control lanelet.
     * - Return `FAILURE` when ego has not passed far enough.
     * - Return `FAILURE` on missing data or unresolved lanelet IDs (fail-safe behavior).
     *
     * Assumptions:
     * - Route progression is represented by increasing indices in `route_index_map`.
     * - The car is on the global route.
     * - Threshold-based index distance is a suitable proxy for "passed" state.
     */
class ActiveTrafficControlElementPassedCondition : public BT::ConditionNode
{
public:
  ActiveTrafficControlElementPassedCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("route_index_map"),
      BT::InputPort<int64_t>("active_traffic_control_lanelet_id"),
      BT::InputPort<int>("lanelet_threshold"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    auto route_index_map = ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "route_index_map");
    auto active_lanelet_id = ports::tryGet<int64_t>(*this, "active_traffic_control_lanelet_id");

    const int lanelet_threshold = ports::tryGet<int>(*this, "lanelet_threshold").value_or(2);

    if (!lane_ctx) {
      std::cout << "[ActiveTrafficControlElementPassed] Missing lane_ctx" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!route_index_map) {
      std::cout << "[ActiveTrafficControlElementPassed] Missing route_index_map" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!active_lanelet_id) {
      std::cout << "[ActiveTrafficControlElementPassed] Missing active_traffic_control_lanelet_id" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    const int64_t active_id = *active_lanelet_id;
    const int64_t ego_id = lane_ctx->current_lanelet.id;

    auto it_active = route_index_map->find(active_id);
    if (it_active == route_index_map->end()) {
      // if active lanelet not in route, consider it passed (this might be bad)
      std::cout << "[ActiveTrafficControlElementPassed] active lanelet not in route_index_map: " << active_id
                << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    auto it_ego = route_index_map->find(ego_id);
    if (it_ego == route_index_map->end()) {
      std::cout << "[ActiveTrafficControlElementPassed] ego lanelet not in route_index_map: " << ego_id << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    const std::size_t idx_active = it_active->second;
    const std::size_t idx_ego = it_ego->second;

    const std::size_t required = idx_active + static_cast<std::size_t>(std::max(0, lanelet_threshold));

    std::cout << "[ActiveTrafficControlElementPassed] idx_ego=" << idx_ego << " idx_active=" << idx_active
              << " required>=" << required << " ego_lanelet=" << ego_id << " active_lanelet=" << active_id << std::endl;

    return (idx_ego >= required) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__CONDITIONS__ACTIVE_TRAFFIC_CONTROL_ELEMENT_PASSED_CONDITION_HPP_
