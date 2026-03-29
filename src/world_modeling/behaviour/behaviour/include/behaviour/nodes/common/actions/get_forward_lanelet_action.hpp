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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_FORWARD_LANELET_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_FORWARD_LANELET_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/msg/route_ahead.hpp"

namespace behaviour
{
class GetForwardLaneletAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetForwardLaneletAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<lanelet_msgs::msg::RouteAhead::SharedPtr>("route_ahead"),
      BT::InputPort<lanelet_msgs::msg::LaneletAhead::SharedPtr>("lanelets_ahead"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("lanelets_ahead_index_map"),
      BT::OutputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("out_lanelet"),
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

    auto route_ahead = ports::tryGetPtr<lanelet_msgs::msg::RouteAhead>(*this, "route_ahead");
    if (route_ahead) {
      const auto route_successor = getForwardLaneletFromRoute(*lane_ctx, *route_ahead);
      if (route_successor) {
        setOutput("out_lanelet", route_successor);
        return BT::NodeStatus::SUCCESS;
      }
    }

    auto lanelets_ahead = ports::tryGetPtr<lanelet_msgs::msg::LaneletAhead>(*this, "lanelets_ahead");
    if (!ports::require(lanelets_ahead, "lanelets_ahead", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto index_map = ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "lanelets_ahead_index_map");
    if (!index_map) {
      index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();
      index_map->reserve(lanelets_ahead->lanelets.size());
      for (std::size_t i = 0; i < lanelets_ahead->lanelets.size(); ++i) {
        (*index_map)[lanelets_ahead->lanelets[i].id] = i;
      }
    }

    for (const auto successor_id : lane_ctx->current_lanelet.successor_ids) {
      const auto it = index_map->find(successor_id);
      if (it == index_map->end() || it->second >= lanelets_ahead->lanelets.size()) {
        continue;
      }

      setOutput("out_lanelet", std::make_shared<lanelet_msgs::msg::Lanelet>(lanelets_ahead->lanelets[it->second]));
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG_STREAM(logger(), "forward_lanelet_not_found current_lanelet_id=" << lane_ctx->current_lanelet.id);
    return BT::NodeStatus::FAILURE;
  }

private:
  static lanelet_msgs::msg::Lanelet::SharedPtr getForwardLaneletFromRoute(
    const lanelet_msgs::msg::CurrentLaneContext & lane_ctx, const lanelet_msgs::msg::RouteAhead & route_ahead)
  {
    if (!route_ahead.has_active_route || route_ahead.ids.empty() || route_ahead.lanelets.empty()) {
      return nullptr;
    }

    const std::size_t n = std::min(route_ahead.ids.size(), route_ahead.lanelets.size());
    for (std::size_t i = 0; i < n; ++i) {
      if (route_ahead.ids[i] != lane_ctx.current_lanelet.id) {
        continue;
      }

      if (i + 1 >= n) {
        return nullptr;
      }

      return std::make_shared<lanelet_msgs::msg::Lanelet>(route_ahead.lanelets[i + 1]);
    }

    if (route_ahead.ids.front() == lane_ctx.current_lanelet.id && n > 1) {
      return std::make_shared<lanelet_msgs::msg::Lanelet>(route_ahead.lanelets[1]);
    }

    return nullptr;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_FORWARD_LANELET_ACTION_HPP_
