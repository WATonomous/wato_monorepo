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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_ROUTE_PREFERRED_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_ROUTE_PREFERRED_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"

namespace behaviour
{

/**
 * @class GetFollowRoutePreferredLaneletsAction
 * @brief SyncActionNode to build preferred lanelets for follow-route behavior.
 *
 * Logic:
 * - Read the route_ahead message from the blackboard.
 * - Extract lanelet IDs from the route ahead as the preferred lanelets.
 */
class GetFollowRoutePreferredLaneletsAction : public BT::SyncActionNode
{
public:
  GetFollowRoutePreferredLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::RouteAhead::SharedPtr>("route_ahead", "Route ahead message"),
      BT::OutputPort<std::vector<int64_t>>(
        "out_preferred_lanelet_ids", "Preferred lanelet IDs for follow-route behaviour"),
      BT::OutputPort<std::string>("error_message", "Error description if the node fails"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GetFollowRoutePreferredLanelets]: Missing " << port_name << " input" << std::endl;
    };

    auto route_ahead = ports::tryGetPtr<lanelet_msgs::msg::RouteAhead>(*this, "route_ahead");
    if (!ports::require(route_ahead, "route_ahead", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (!route_ahead->has_active_route || route_ahead->ids.empty()) {
      std::cout << "[GetFollowRoutePreferredLanelets]: No active route or empty route ahead" << std::endl;
      setOutput("error_message", std::string("No active route or empty route ahead"));
      return BT::NodeStatus::FAILURE;
    }

    std::vector<int64_t> preferred(route_ahead->ids.begin(), route_ahead->ids.end());

    setOutput("out_preferred_lanelet_ids", preferred);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_ROUTE_PREFERRED_LANELETS_ACTION_HPP_