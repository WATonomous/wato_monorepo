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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <memory>

#include "behaviour/stop_sign_arrival_queue.hpp"

namespace behaviour
{
/**
 * @class ClearStopSignArrivalQueueAction
 * @brief SyncActionNode to clear the stop sign arrival queue.
 */
class ClearStopSignArrivalQueueAction : public BT::SyncActionNode
{
public:
  ClearStopSignArrivalQueueAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<StopSignArrivalQueue::SharedPtr>("arrival_queue"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::cout << "[ClearStopSignArrivalQueue]: Clearing arrival queue" << std::endl;
    auto cleared_queue = std::make_shared<StopSignArrivalQueue>();
    setOutput("arrival_queue", cleared_queue);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_
