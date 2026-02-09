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

#ifndef BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_
#define BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviour/area_occupancy_store.hpp"
#include "behaviour/utils/ports.hpp"
#include "behaviour/utils/types.hpp"

namespace behaviour
{
  /**
   * @class SafeLaneChangeCondition
   * @brief ConditionNode to check whether lane-change occupancy areas are clear.
   *
   * Logic:
   * - Select configured occupancy areas based on lane-change direction.
   * - Return `FAILURE` when transition or snapshot is missing.
   * - Return `FAILURE` for non-lane-change transitions (`SUCCESSOR`).
   * - Return `FAILURE` if any checked area is occupied.
   * - Return `SUCCESS` only when all relevant areas are clear.
   *
   * Assumptions:
   * - Occupancy area names match world-model configuration.
   * - Area occupancy snapshot is recent enough for lane-change gating.
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
          BT::InputPort<std::vector<std::string>>("left_lane_change_areas", "Occupancy areas to check for left lane change"),
          BT::InputPort<std::vector<std::string>>("right_lane_change_areas", "Occupancy areas to check for right lane change"),
      };
    };

    BT::NodeStatus tick() override
    {
      const auto missing_input_callback = [&](const char *port_name)
      { std::cout << "[SafeLaneChange]: Missing " << port_name << " input" << std::endl; };

      auto transition = ports::tryGet<types::LaneTransition>(*this, "lane_transition");
      if (!ports::require(transition, "lane_transition", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto area_snapshot = ports::tryGetPtr<const AreaOccupancyStore::Snapshot>(*this, "area_occupancy_snapshot");
      if (!ports::require(area_snapshot, "area_occupancy_snapshot", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto left_areas = ports::tryGet<std::vector<std::string>>(*this, "left_lane_change_areas");
      if (!ports::require(left_areas, "left_lane_change_areas", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }

      auto right_areas = ports::tryGet<std::vector<std::string>>(*this, "right_lane_change_areas");
      if (!ports::require(right_areas, "right_lane_change_areas", missing_input_callback))
      {
        return BT::NodeStatus::FAILURE;
      }
      if (transition == types::LaneTransition::SUCCESSOR)
      {
        std::cout << "[SafeLaneChange]: Transition is SUCCESSOR (no lane change)" << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      // determine which areas to check based on lane change direction
      const std::vector<std::string> &areas =
          (transition == types::LaneTransition::LEFT) ? *left_areas : *right_areas;

      if (areas.empty())
      {
        std::cout << "[SafeLaneChange]: No configured occupancy areas for transition " << types::toString(*transition)
                  << std::endl;
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
  };
} // namespace behaviour

#endif // BEHAVIOUR__NODES__LANE_NAVIGATION__CONDITIONS__SAFE_LANE_CHANGE_CONDITION_HPP_
