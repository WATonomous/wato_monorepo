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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <string>
#include <vector>

namespace behaviour
{
    /**
     * @brief Clears stop sign priority ids and resets the latch flag.
     */
    class ClearStopSignPriorityCarsAction : public BT::SyncActionNode
    {
    public:
        ClearStopSignPriorityCarsAction(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<std::vector<std::string>>("priority_car_ids"),
                BT::OutputPort<bool>("priority_latched"),
            };
        }

        BT::NodeStatus tick() override
        {
            std::cout << "[ClearStopSignPriorityCarsAction]: Clearing stop sign priority car ids" << std::endl;
            setOutput("priority_car_ids", std::vector<std::string>{});
            setOutput("priority_latched", false);
            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__CLEAR_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_
