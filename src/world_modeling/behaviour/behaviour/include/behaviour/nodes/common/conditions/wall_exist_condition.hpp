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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_EXIST_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_EXIST_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "behaviour/utils/ports.hpp"

namespace behaviour
{
/**
 * @class WallIdExistCondition
 * @brief ConditionNode to check whether a wall_id is available.
 */
class WallIdExistCondition : public BT::ConditionNode
{
public:
  WallIdExistCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int32_t>("wall_id"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[WallIdValid] Missing " << port_name << " input -> treat as no-op" << std::endl;
    };

    auto wall_id = ports::tryGet<int32_t>(*this, "wall_id");
    if (!ports::require(wall_id, "wall_id", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;  // used for gating in a Fallback
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__WALL_EXIST_CONDITION_HPP_
