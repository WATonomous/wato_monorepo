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

#ifndef BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_
#define BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <iostream>
#include <string>
#include <vector>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
    /**
     * @brief Latches stop sign priority car ids exactly once until explicitly cleared.
     *
     * Uses a separate bool flag (priority_latched) to distinguish:
     * - "never latched yet" vs
     * - "latched (possibly empty)"
     */
    class SetStopSignPriorityCarsAction : public BT::SyncActionNode
    {
    public:
        SetStopSignPriorityCarsAction(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<std::string>>("current_car_ids"),
                BT::InputPort<std::vector<std::string>>("priority_car_ids"),
                BT::InputPort<bool>("priority_latched"),

                BT::OutputPort<std::vector<std::string>>("out_priority_car_ids"),
                BT::OutputPort<bool>("out_priority_latched"),
            };
        }

        BT::NodeStatus tick() override
        {
            // If already latched, keep returning the stored ids.
            const bool latched = ports::tryGet<bool>(*this, "priority_latched").value_or(false);
            if (latched)
            {
                auto existing = ports::tryGet<std::vector<std::string>>(*this, "priority_car_ids");
                if (!existing)
                {
                    std::cout << "[SetStopSignPriorityCarsAction] priority_latched=true but priority_car_ids missing" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                setOutput("out_priority_car_ids", *existing);
                setOutput("out_priority_latched", true);
                return BT::NodeStatus::SUCCESS;
            }

            // Not latched yet: latch current set (may be empty).
            auto current = ports::tryGet<std::vector<std::string>>(*this, "current_car_ids");
            if (!current)
            {
                std::cout << "[SetStopSignPriorityCarsAction] Missing current_car_ids" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            setOutput("out_priority_car_ids", *current);
            setOutput("out_priority_latched", true);
            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__NODES__INTERSECTION__ACTIONS__SET_STOP_SIGN_PRIORITY_CARS_ACTION_HPP_
