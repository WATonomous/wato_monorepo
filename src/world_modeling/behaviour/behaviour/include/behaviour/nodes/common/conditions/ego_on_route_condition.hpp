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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_ROUTE_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_ROUTE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class EgoOnRouteCondition
 * @brief ConditionNode to check whether ego lanelet exists on the route.
 */
class EgoOnRouteCondition : public BT::ConditionNode
{
public:
  EgoOnRouteCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, size_t>>>("route_index_map"),
      BT::InputPort<std::shared_ptr<lanelet_msgs::msg::CurrentLaneContext>>("lane_ctx")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[EgoOnRoute]: Missing " << port_name << " input" << std::endl;
    };

    auto route_index_map = ports::tryGetPtr<std::unordered_map<int64_t, size_t>>(*this, "route_index_map");
    if (!ports::require(route_index_map, "route_index_map", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if current lanelet ID exists in the route index map
    auto it = route_index_map->find(ctx->current_lanelet.id);

    // log for debugging
    (it != route_index_map->end())
      ? std::cout << "[EgoOnRoute]: Current lanelet " << ctx->current_lanelet.id << " is on the route" << std::endl
      : std::cout << "[EgoOnRoute]: Current lanelet " << ctx->current_lanelet.id << " is not on the route" << std::endl;

    return (it != route_index_map->end()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__EGO_ON_ROUTE_CONDITION_HPP_
