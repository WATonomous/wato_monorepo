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

#ifndef BEHAVIOUR__DETERMINE_MANEUVER_NODE_HPP_
#define BEHAVIOUR__DETERMINE_MANEUVER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "behaviour/types/lattice_planner_behaviour.hpp"
#include "world_modeling_msgs/msg/lanelet.hpp"

namespace behaviour
{
  /**
   * @class DetermineManeuverAction
   * @brief BT node to determine the next maneuver based on the current position and global route.
   */
  class ComputeManeuverAction : public BT::SyncActionNode
  {
  public:
    ComputeManeuverAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int64_t>("current_lanelet_id", "Current lanelet ID of the vehicle"),
          BT::InputPort<std::shared_ptr<std::vector<world_modeling_msgs::msg::Lanelet>>>(
              "path", "Pointer to the list of lanelets in the global route"),
          BT::InputPort<std::shared_ptr<std::vector<uint8_t>>>(
              "transitions", "Pointer to the list of transitions between lanelets"),
          BT::OutputPort<LatticePlannerBehaviour>(
              "maneuver",
              "The determined maneuver: 'follow_lane', 'lane_change_left', 'lane_change_right', '"),
          BT::OutputPort<int64_t>("target_lanelet_id", "Target lanelet ID for the next maneuver")};
    }

    BT::NodeStatus tick() override
    {
      auto current_lanelet_id = getInput<int64_t>("current_lanelet_id");
      auto global_path = getInput<std::shared_ptr<std::vector<world_modeling_msgs::msg::Lanelet>>>("path");
      auto transitions = getInput<std::shared_ptr<std::vector<uint8_t>>>("transitions");

      // validate port
      if (!current_lanelet_id || !global_path || !transitions)
      {
        return BT::NodeStatus::FAILURE;
      }

      // validate pointers
      if (!global_path.value() || !transitions.value())
      {
        return BT::NodeStatus::FAILURE;
      }

      // try to find current lanelet in the path
      auto it = std::find_if(global_path.value()->begin(), global_path.value()->end(), [&](const auto &lanelet)
                             { return lanelet.id == current_lanelet_id.value(); });

      // if we don't find it, either care out track, or no path

      if (it == global_path.value()->end())
      {
        return BT::NodeStatus::FAILURE;
      }

      size_t index = std::distance(global_path.value()->begin(), it);

      if (index >= transitions.value()->size())
      {
        return BT::NodeStatus::FAILURE;
      }

      uint8_t trans = transitions.value()->at(index);

      // Transition mapping: 0=SUCCESSOR, 1=LEFT, 2=RIGHT
      if (trans == 1)
      {
        setOutput("maneuver", LatticePlannerBehaviour::LANE_CHANGE_LEFT);
      }
      else if (trans == 2)
      {
        setOutput("maneuver", LatticePlannerBehaviour::LANE_CHANGE_RIGHT);
      }
      else
      {
        setOutput("maneuver", LatticePlannerBehaviour::FOLLOW_EGO_LANE);
      }

      size_t target_index = (index + 1 < global_path.value()->size()) ? index + 1 : index;
      setOutput("target_lanelet_id", global_path.value()->at(target_index).id);

      return BT::NodeStatus::SUCCESS;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__COMPUTE_MANEUVER_NODE_HPP_
