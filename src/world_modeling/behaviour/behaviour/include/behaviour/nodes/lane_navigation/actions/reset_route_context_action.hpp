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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__RESET_ROUTE_CONTEXT_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__RESET_ROUTE_CONTEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class ResetRouteContextAction
 * @brief Clears goal and route-related blackboard entries so route/goal guards fail on the next tick.
 */
class ResetRouteContextAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  ResetRouteContextAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("out_goal_point"),
      BT::OutputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("out_goal_lanelet"),
      BT::OutputPort<lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr>("out_global_route"),
      BT::OutputPort<std::shared_ptr<std::unordered_map<int64_t, size_t>>>("out_global_route_index_map"),
      BT::OutputPort<std::vector<int64_t>>("out_global_route_lanelet_ids"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PointStamped::SharedPtr null_point = nullptr;
    lanelet_msgs::msg::Lanelet::SharedPtr null_lanelet = nullptr;
    lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr null_route = nullptr;
    std::shared_ptr<std::unordered_map<int64_t, size_t>> null_route_index_map = nullptr;

    setOutput("out_goal_point", null_point);
    setOutput("out_goal_lanelet", null_lanelet);
    setOutput("out_global_route", null_route);
    setOutput("out_global_route_index_map", null_route_index_map);
    setOutput("out_global_route_lanelet_ids", std::vector<int64_t>{});

    RCLCPP_DEBUG_STREAM(logger(), "Cleared goal point, goal lanelet, and global route context" );
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__RESET_ROUTE_CONTEXT_ACTION_HPP_
