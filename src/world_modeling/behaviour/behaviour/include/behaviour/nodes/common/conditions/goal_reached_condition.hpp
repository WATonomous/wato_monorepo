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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <iostream>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace behaviour
{
/**
   * @class GoalReachedCondition
   * @brief ConditionNode to check whether ego is within goal distance threshold.
   */
class GoalReachedCondition : public BT::ConditionNode
{
public:
  GoalReachedCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("current_point"),
      BT::InputPort<geometry_msgs::msg::Point::SharedPtr>("goal_point"),
      BT::InputPort<double>("threshold_m")};
  };

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GoalReached]: Missing " << port_name << " input" << std::endl;
    };

    geometry_msgs::msg::Point::SharedPtr gp = ports::tryGetPtr<geometry_msgs::msg::Point>(*this, "goal_point");
    if (!ports::require(gp, "goal_point", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Point::SharedPtr cp = ports::tryGetPtr<geometry_msgs::msg::Point>(*this, "current_point");
    if (!ports::require(cp, "current_point", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    double threshold = ports::tryGet<double>(*this, "threshold_m").value_or(1.0);

    // Calculate 2D distance
    double dx = gp->x - cp->x;
    double dy = gp->y - cp->y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Check if within threshold
    if (distance <= threshold) {
      std::cout << "[GoalReached]: Distance " << distance << " is within threshold " << threshold << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    std::cout << "[GoalReached]: Distance " << distance << " is outside threshold " << threshold << std::endl;
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour
#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_
