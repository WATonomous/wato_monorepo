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

#ifndef BEHAVIOUR__NODES__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_
#define BEHAVIOUR__NODES__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <memory>
#include <string>

#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/current_lane_context.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"

namespace behaviour
{
/**
 * @class SafeLaneChangeCondition
 * @brief BT condition node that checks for cars in current and target lanelets.
 *
 * Returns SUCCESS if no cars are found in either the current lanelet or target lanelet.
 */
class SafeLaneChangeCondition : public BT::ConditionNode
{
public:
  SafeLaneChangeCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<lanelet_msgs::msg::CurrentLaneContext::SharedPtr>(
        "lane_ctx", "Current lane context"),
      BT::InputPort<std::shared_ptr<lanelet_msgs::msg::Lanelet>>(
        "target_lanelet", "Lanelet being considered for lane change"),
      BT::InputPort<std::shared_ptr<const DynamicObjectStore::Snapshot>>(
        "dynamic_objects_snapshot", "Dynamic object snapshot from blackboard"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto lane_ctx = ports::tryGetPtr<lanelet_msgs::msg::CurrentLaneContext>(*this, "lane_ctx");
    auto target_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "target_lanelet");
    auto snapshot = ports::tryGetPtr<const DynamicObjectStore::Snapshot>(*this, "dynamic_objects_snapshot");

    if (!lane_ctx) {
      std::cout << "[SafeLaneChange]: Missing lane context" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!target_lanelet) {
      std::cout << "[SafeLaneChange]: Missing target lanelet" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!snapshot || !snapshot->objects_snapshot_) {
      std::cout << "[SafeLaneChange]: Missing dynamic object snapshot" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // this only gets the ids of cars in the target lanelet and current lanelet (this could be bad if target lanelet is close)
    // essentially this doesn't look at the disatnce of the cars, just if there are any cars in those lanelets
    // TODO(wato): use the area occupancy published by world modeling when up
    const int64_t current_id = lane_ctx->current_lanelet.id;
    const int64_t target_id = target_lanelet->id;

    const auto & current_idxs = snapshot->getCarIndicesInLanelet(current_id);
    const auto & target_idxs = snapshot->getCarIndicesInLanelet(target_id);

    if (!current_idxs.empty() || !target_idxs.empty()) {
      std::cout << "[SafeLaneChange]: Not safe (cars detected in lanelets)" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SafeLaneChange]: Safe (no cars detected)" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_
