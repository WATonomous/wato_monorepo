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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{
/**
 * @class GetLaneChangePreferredLaneletsAction
 * @brief SyncActionNode to build preferred lanelets for lane-change behavior.
 *
 * Logic:
 * - Start preferred lanelets with target lanelet.
 * - Append target-lanelet successors in order.
 */
class GetLaneChangePreferredLaneletsAction : public BT::SyncActionNode
{
public:
  GetLaneChangePreferredLaneletsAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>("lane_ctx", "Current lane context"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("target_lanelet", "Target lanelet for lane change"),
      BT::OutputPort<std::vector<int64_t>>(
        "out_preferred_lanelet_ids", "Preferred lanelet IDs for behaviour execution"),
      BT::OutputPort<std::string>("error_message", "Error description if the node fails"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GetLaneChangePreferredLanelets]: Missing " << port_name << " input" << std::endl;
    };

    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    if (!ports::require(lane_ctx, "lane_ctx", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto target_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "target_lanelet");
    if (!ports::require(target_lanelet, "target_lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    std::vector<int64_t> preferred;
    preferred.push_back(target_lanelet->id);
    for (const auto & id : target_lanelet->successor_ids) {
      preferred.push_back(id);
    }

    setOutput("out_preferred_lanelet_ids", preferred);
    std::cout << "[GetLaneChangePreferredLanelets]: Result=SUCCESS (preferred_ids_count=" << preferred.size() << ")"
              << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__LANE_NAVIGATION__ACTIONS__GET_LANE_CHANGE_PREFERRED_LANELETS_ACTION_HPP_
