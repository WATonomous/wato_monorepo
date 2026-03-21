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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__GLOBAL_ROUTE_EXIST_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__GLOBAL_ROUTE_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class GlobalRouteExistCondition
 * @brief ConditionNode to check whether a route exists and is non-empty.
 */
class GlobalRouteExistCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  GlobalRouteExistCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::shared_ptr<lanelet_msgs::srv::GetShortestRoute::Response>>("route")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "route");
    if (!ports::require(route, "route", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (route->lanelets.empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "Route does not exist or is empty");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GLOBAL_ROUTE_EXIST_CONDITION_HPP_
