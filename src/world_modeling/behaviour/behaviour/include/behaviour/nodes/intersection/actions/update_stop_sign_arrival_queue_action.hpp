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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__UPDATE_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__UPDATE_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/time.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviour/stop_sign_arrival_queue.hpp"
#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class UpdateStopSignArrivalQueueAction
 * @brief SyncActionNode to update the stop sign arrival queue with current detections.
 *
 * This action maintains a persistent queue of vehicles that have arrived at the stop sign.
 * It tracks arrival times and which vehicles have proceeded.
 */
class UpdateStopSignArrivalQueueAction : public BT::SyncActionNode
{
public:
  UpdateStopSignArrivalQueueAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::pair<std::string, rclcpp::Time>>>("current_cars_with_timestamps"),
      BT::InputPort<double>("simultaneous_arrival_threshold_s"),
      BT::BidirectionalPort<StopSignArrivalQueue::SharedPtr>("arrival_queue"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto current_cars =
      ports::tryGet<std::vector<std::pair<std::string, rclcpp::Time>>>(*this, "current_cars_with_timestamps");
    auto threshold_s = ports::tryGet<double>(*this, "simultaneous_arrival_threshold_s").value_or(1.0);

    if (!current_cars) {
      std::cout << "[UpdateStopSignArrivalQueue] Missing current_cars_with_timestamps" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // Get or create arrival queue
    auto queue = ports::tryGetPtr<StopSignArrivalQueue>(*this, "arrival_queue");
    if (!queue) {
      // Create new queue
      queue = std::make_shared<StopSignArrivalQueue>();
      setOutput("arrival_queue", queue);
    }

    // Update the queue with current detections
    queue->update(*current_cars, threshold_s);

    std::cout << "[UpdateStopSignArrivalQueue] Queue size: " << queue->size() << std::endl;

    // Log current queue state for debugging
    const auto & records = queue->records();
    for (size_t i = 0; i < records.size(); ++i) {
      const auto & rec = records[i];
      std::cout << "  [" << i << "] ID: " << rec.car_id << ", arrived: " << rec.arrival_time.seconds()
                << "s, stopped: " << (rec.stopped ? "yes" : "no")
                << ", proceeded: " << (rec.has_proceeded ? "yes" : "no") << std::endl;
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__UPDATE_STOP_SIGN_ARRIVAL_QUEUE_ACTION_HPP_
