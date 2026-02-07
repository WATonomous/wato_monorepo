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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__MARK_EGO_PROCEEDED_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__MARK_EGO_PROCEEDED_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <memory>
#include <string>

#include "behaviour/stop_sign_arrival_queue.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class MarkEgoProceededAction
 * @brief SyncActionNode to mark ego vehicle as having proceeded through the intersection.
 */
class MarkEgoProceededAction : public BT::SyncActionNode
{
public:
  MarkEgoProceededAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<StopSignArrivalQueue::SharedPtr>("arrival_queue"),
      BT::InputPort<std::string>("ego_id"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto queue = ports::tryGetPtr<StopSignArrivalQueue>(*this, "arrival_queue");
    auto ego_id = ports::tryGet<std::string>(*this, "ego_id");

    if (!queue) {
      std::cout << "[MarkEgoProceeded] Missing arrival_queue" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    if (!ego_id) {
      std::cout << "[MarkEgoProceeded] Missing ego_id" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    queue->markProceeded(*ego_id);
    std::cout << "[MarkEgoProceeded] Marked ego (" << *ego_id << ") as proceeded" << std::endl;

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__MARK_EGO_PROCEEDED_ACTION_HPP_
