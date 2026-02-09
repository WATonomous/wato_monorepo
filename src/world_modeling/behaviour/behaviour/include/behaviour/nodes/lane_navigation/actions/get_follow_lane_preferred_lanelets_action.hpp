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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{

/**
 * @class GetFollowLanePreferredLaneletsAction
 * @brief SyncActionNode to build preferred lanelets for follow-lane behavior.
 *
 * Logic:
 * - Start preferred lanelets with current lanelet.
 * - Append current-lanelet successors in order.
 */
class GetFollowLanePreferredLaneletsAction : public BT::SyncActionNode
{
public:
  GetFollowLanePreferredLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx", "Current lane context"),
      BT::OutputPort<std::vector<int64_t>>("out_preferred_lanelet_ids", "Preferred lanelet IDs (straight only)"),
      BT::OutputPort<std::string>("error_message", "Error if failed"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!lane_ctx) {
      setOutput("error_message", "missing_lane_context");
      std::cout << "[GetFollowLanePreferredLanelets]: Missing lane_ctx input" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::vector<int64_t> preferred;
    preferred.push_back(lane_ctx->current_lanelet.id);
    for (const auto & id : lane_ctx->current_lanelet.successor_ids) {
      preferred.push_back(id);
    }

    setOutput("out_preferred_lanelet_ids", preferred);
    std::cout << "[GetFollowLanePreferredLanelets]: Result=SUCCESS (preferred_ids_count=" << preferred.size() << ")"
              << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_FOLLOW_LANE_PREFERRED_LANELETS_ACTION_HPP_
