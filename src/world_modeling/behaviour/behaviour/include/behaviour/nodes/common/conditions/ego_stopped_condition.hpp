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

#ifndef BEHAVIOUR__EGO_STOPPED_CONDITION_HPP_
#define BEHAVIOUR__EGO_STOPPED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cmath>
#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"

namespace behaviour
{

    class EgoStoppedCondition : public BT::ConditionNode
    {
    public:
        EgoStoppedCondition(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<double>("ego_velocity"),
                BT::InputPort<double>("threshold_velocity"),
            };
        }

        BT::NodeStatus tick() override
        {
            auto ego_vel = ports::tryGet<double>(*this, "ego_velocity");
            const double thresh = ports::tryGet<double>(*this, "threshold_velocity").value_or(0.1);

            if (!ego_vel)
            {
                std::cout << "[EgoStopped]: Missing or invalid ego_velocity" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            const double v = *ego_vel;

            if (std::fabs(v) <= thresh)
            {
                std::cout << "[EgoStopped]: ego_velocity " << v
                          << " <= threshold_velocity " << thresh << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            std::cout << "[EgoStopped]: ego_velocity " << v
                      << " > threshold_velocity " << thresh << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    };

} // namespace behaviour

#endif // BEHAVIOUR__EGO_STOPPED_CONDITION_HPP_
