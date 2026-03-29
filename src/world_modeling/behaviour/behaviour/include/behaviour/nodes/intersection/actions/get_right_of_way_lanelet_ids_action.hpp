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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_RIGHT_OF_WAY_LANELET_IDS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_RIGHT_OF_WAY_LANELET_IDS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/lanelet.hpp"
#include "behaviour/utils/ports.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"

namespace behaviour
{
class GetRightOfWayLaneletIdsAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  GetRightOfWayLaneletIdsAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx"),
      BT::OutputPort<std::vector<int64_t>>("out_lanelet_ids"),
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

    std::vector<int64_t> ego_candidate_lanelet_ids;
    ego_candidate_lanelet_ids.push_back(lane_ctx->current_lanelet.id);
    for (const auto lanelet_id : lane_ctx->upcoming_lanelet_ids) {
      if (!utils::lanelet::containsLaneletId(ego_candidate_lanelet_ids, lanelet_id)) {
        ego_candidate_lanelet_ids.push_back(lanelet_id);
      }
    }

    std::vector<int64_t> right_of_way_lanelet_ids;
    for (const auto & reg_elem : lane_ctx->current_lanelet.regulatory_elements) {
      if (reg_elem.subtype != "right_of_way") {
        continue;
      }

      bool ego_matches_yield_lanelet = false;
      for (const auto ego_lanelet_id : ego_candidate_lanelet_ids) {
        if (utils::lanelet::containsLaneletId(reg_elem.yield_lanelet_ids, ego_lanelet_id)) {
          ego_matches_yield_lanelet = true;
          break;
        }
      }

      if (!ego_matches_yield_lanelet) {
        continue;
      }

      if (reg_elem.right_of_way_lanelet_ids.empty()) {
        continue;
      }
      for (const auto lanelet_id : reg_elem.right_of_way_lanelet_ids) {
        if (utils::lanelet::containsLaneletId(ego_candidate_lanelet_ids, lanelet_id)) {
          continue;
        }
        if (!utils::lanelet::containsLaneletId(right_of_way_lanelet_ids, lanelet_id)) {
          right_of_way_lanelet_ids.push_back(lanelet_id);
        }
      }
    }

    if (right_of_way_lanelet_ids.empty()) {
      RCLCPP_DEBUG_STREAM(logger(), "empty right of way lanelet_ids for lanelet_id=" << lane_ctx->current_lanelet.id);
    }

    setOutput("out_lanelet_ids", right_of_way_lanelet_ids);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__GET_RIGHT_OF_WAY_LANELET_IDS_ACTION_HPP_
