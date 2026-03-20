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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELET_BY_RELATION_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELET_BY_RELATION_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include "behaviour/nodes/bt_logger_base.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"

namespace behaviour
{
/**
 * @class GetLaneletByRelationAction
 * @brief Resolves the adjacent left or right lanelet using the lanelets-ahead cache.
 */
class GetLaneletByRelationAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetLaneletByRelationAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::InputPort<lanelet_msgs::msg::LaneletAhead::SharedPtr>("lanelets_ahead"),
      BT::InputPort<std::shared_ptr<std::unordered_map<int64_t, std::size_t>>>("lanelets_ahead_index_map"),
      BT::InputPort<types::LaneTransition>("relation"),
      BT::OutputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("out_lanelet"),
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

    auto lanelets_ahead = ports::tryGetPtr<lanelet_msgs::msg::LaneletAhead>(*this, "lanelets_ahead");
    if (!ports::require(lanelets_ahead, "lanelets_ahead", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto relation = ports::tryGet<types::LaneTransition>(*this, "relation");
    if (!ports::require(relation, "relation", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (*relation == types::LaneTransition::SUCCESSOR) {
      RCLCPP_DEBUG_STREAM(logger(), "SUCCESSOR is not a lateral lane relation" );
      return BT::NodeStatus::FAILURE;
    }

    auto index_map =
      ports::tryGetPtr<std::unordered_map<int64_t, std::size_t>>(*this, "lanelets_ahead_index_map");
    if (!index_map) {
      index_map = std::make_shared<std::unordered_map<int64_t, std::size_t>>();
      index_map->reserve(lanelets_ahead->lanelets.size());
      for (std::size_t i = 0; i < lanelets_ahead->lanelets.size(); ++i) {
        (*index_map)[lanelets_ahead->lanelets[i].id] = i;
      }
    }

    const int64_t target_lanelet_id = (*relation == types::LaneTransition::LEFT)
                                        ? lane_ctx->current_lanelet.left_lane_id
                                        : lane_ctx->current_lanelet.right_lane_id;
    if (target_lanelet_id <= 0) {
      RCLCPP_DEBUG_STREAM(logger(), "No adjacent lanelet for relation "
                << types::toString(*relation) );
      return BT::NodeStatus::FAILURE;
    }

    const auto target_it = index_map->find(target_lanelet_id);
    if (target_it == index_map->end() || target_it->second >= lanelets_ahead->lanelets.size()) {
      RCLCPP_DEBUG_STREAM(logger(), "Lanelet " << target_lanelet_id
                << " not found in lanelets_ahead cache" );
      return BT::NodeStatus::FAILURE;
    }

    setOutput(
      "out_lanelet",
      std::make_shared<lanelet_msgs::msg::Lanelet>(lanelets_ahead->lanelets[target_it->second]));
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_LANELET_BY_RELATION_ACTION_HPP_
