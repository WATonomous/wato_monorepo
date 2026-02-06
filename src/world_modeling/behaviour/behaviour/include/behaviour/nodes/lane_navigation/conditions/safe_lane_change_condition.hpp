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

#include "behaviour/area_occupancy_store.hpp"
#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/types.hpp"
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
        SafeLaneChangeCondition(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<types::LaneTransition>("lane_transition", "Lane transition direction"),
                BT::InputPort<std::shared_ptr<const AreaOccupancyStore::Snapshot>>(
                    "area_occupancy_snapshot", "Area occupancy snapshot from blackboard"),
            };
        };

        BT::NodeStatus
        tick() override
        {
            auto transition = ports::tryGet<types::LaneTransition>(*this, "lane_transition");
            auto area_snapshot = ports::tryGetPtr<const AreaOccupancyStore::Snapshot>(*this, "area_occupancy_snapshot");

            if (!transition)
            {
                std::cout << "[SafeLaneChange]: Missing lane_transition" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            if (!area_snapshot)
            {
                std::cout << "[SafeLaneChange]: Missing area_occupancy_snapshot" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            if (transition == types::LaneTransition::SUCCESSOR)
            {
                std::cout << "[SafeLaneChange]: Transition is SUCCESSOR (no lane change)" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            // determine which areas to check based on lane change direction
            const std::vector<std::string> &areas =
                (transition == types::LaneTransition::LEFT) ? left_lane_change_areas_ : right_lane_change_areas_;

            if (areas.empty())
            {
                std::cout << "[SafeLaneChange]: No configured occupancy areas for transition "
                          << types::toString(*transition) << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            for (const auto &area_name : areas)
            {
                if (area_snapshot->isOccupied(area_name))
                {
                    std::cout << "[SafeLaneChange]: Not safe (occupied area=" << area_name
                              << ", transition=" << types::toString(*transition) << ")" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }

            std::cout << "[SafeLaneChange]: Safe (areas clear, transition=" << types::toString(*transition) << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    private:
        // must match the names configured in world_model's occupancy_areas parameters
        // TODO(wato): add safer areas here and in config (e.g. larger area that covers the whole area needed for lane change)
        std::vector<std::string> left_lane_change_areas_{"left_lane_change_corridor"};
        std::vector<std::string> right_lane_change_areas_{"right_lane_change_corridor"};
    };
} // namespace behaviour

#endif // BEHAVIOUR__NODES__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_
