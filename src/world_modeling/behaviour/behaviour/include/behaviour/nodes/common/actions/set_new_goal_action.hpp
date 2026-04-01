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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__SET_NEW_GOAL_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__SET_NEW_GOAL_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"

namespace behaviour
{
/**
 * @class SetNewGoalAction
 * @brief Commits the pending goal and staged route outputs into the active route context.
 */
class SetNewGoalAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetNewGoalAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PointStamped::SharedPtr>("goal_point"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("goal_lanelet"),
      BT::InputPort<lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr>("global_route"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("global_route_index_map"),
      BT::InputPort<std::vector<int64_t>>("global_route_lanelet_ids"),
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("out_goal_point"),
      BT::OutputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("out_goal_lanelet"),
      BT::OutputPort<lanelet_msgs::srv::GetShortestRoute::Response::SharedPtr>("out_global_route"),
      BT::OutputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("out_global_route_index_map"),
      BT::OutputPort<std::vector<int64_t>>("out_global_route_lanelet_ids"),
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("out_pending_goal_point"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_ERROR(logger(), "Missing input port: %s", port_name);
    };

    auto goal_point = ports::tryGetPtr<geometry_msgs::msg::PointStamped>(*this, "goal_point");
    if (!ports::require(goal_point, "goal_point", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto goal_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "goal_lanelet");
    if (!ports::require(goal_lanelet, "goal_lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto global_route = ports::tryGetPtr<lanelet_msgs::srv::GetShortestRoute::Response>(*this, "global_route");
    if (!ports::require(global_route, "global_route", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto global_route_index_map =
      ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "global_route_index_map");
    if (!ports::require(global_route_index_map, "global_route_index_map", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto global_route_lanelet_ids = ports::tryGet<std::vector<int64_t>>(*this, "global_route_lanelet_ids");
    if (!ports::require(global_route_lanelet_ids, "global_route_lanelet_ids", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("out_goal_point", goal_point);
    setOutput("out_goal_lanelet", goal_lanelet);
    setOutput("out_global_route", global_route);
    setOutput("out_global_route_index_map", global_route_index_map);
    setOutput("out_global_route_lanelet_ids", global_route_lanelet_ids.value());
    geometry_msgs::msg::PointStamped::SharedPtr cleared_pending_goal_point = nullptr;
    setOutput("out_pending_goal_point", cleared_pending_goal_point);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__SET_NEW_GOAL_ACTION_HPP_
