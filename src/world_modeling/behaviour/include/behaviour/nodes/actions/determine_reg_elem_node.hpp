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

#ifndef BEHAVIOUR__NODES__DETERMINE_REG_ELEM_NODE_HPP_
#define BEHAVIOUR__NODES__DETERMINE_REG_ELEM_NODE_HPP_

#include <behaviortree_cpp/action_node.h>

#include <string>

// msg
#include "world_modeling_msgs/msg/current_lane_context.hpp"

namespace wato::world_modeling::behaviour
{
  /**
   * @class DetermineRegElemNode
   * @brief BT node to detect upcoming regulatory elements (intersections, traffic lights, etc.)
   * based on the current lane context and upcoming path.
   */
  class DetermineRegElemNode : public BT::SyncActionNode
  {
  public:
    DetermineRegElemNode(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int64_t>("lanelet_id"),

          // Output Ports
          BT::OutputPort<std::string>(
              "reg_elem_type", "Type of the upcoming element (intersection, traffic_light, stop_line, yield_sign, none)"),
      };
    }

    /**
     * @brief Continuously checks the successor lanelet in the current route for regulatory flags.
     */
    BT::NodeStatus tick() override
    {
      auto lanelet_id = getInput<int64_t>("lanelet_id");

      if (!lanelet_id)
      {
        return BT::NodeStatus::FAILURE;
      }

      // pass next lanelet?
      // check which reg elem it has
      // return the type of the reg elem

      // the input will just have the successor lanelet from the current one

      return BT::NodeStatus::SUCCESS;
    }
  };
} // namespace wato::world_modeling::behaviour

#endif // BEHAVIOUR__NODES__DETERMINE_REG_ELEM_NODE_HPP_
