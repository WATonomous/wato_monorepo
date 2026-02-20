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

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "behaviour/utils/utils.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
      BT::InputPort<nav_msgs::msg::Odometry::SharedPtr>("ego_odom"),
      BT::InputPort<lanelet_msgs::msg::Lanelet::SharedPtr>("goal_lanelet"),
      BT::InputPort<double>("threshold_m")};
  };

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      std::cout << "[GoalReached]: Missing " << port_name << " input" << std::endl;
    };

    auto ego_odom = ports::tryGetPtr<nav_msgs::msg::Odometry>(*this, "ego_odom");
    if (!ports::require(ego_odom, "ego_odom", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    auto goal_lanelet = ports::tryGetPtr<lanelet_msgs::msg::Lanelet>(*this, "goal_lanelet");
    if (!ports::require(goal_lanelet, "goal_lanelet", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }
    if (goal_lanelet->centerline.empty()) {
      std::cout << "[GoalReached]: goal_lanelet centerline is empty" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    auto threshold_m = ports::tryGet<double>(*this, "threshold_m");
    if (!ports::require(threshold_m, "threshold_m", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    // Goal-point distance check removed: we now measure distance to end of goal lanelet centerline.
    const auto & cp = ego_odom->pose.pose.position;
    const auto & lanelet_end_point = goal_lanelet->centerline.back();
    const double threshold = *threshold_m;

    // Calculate 2D distance
    const double dx = lanelet_end_point.x - cp.x;
    const double dy = lanelet_end_point.y - cp.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    // Check if within threshold
    if (distance <= threshold) {
      std::cout << "[GoalReached]: Distance " << distance << " is within threshold " << threshold
                << " (goal_lanelet_id=" << goal_lanelet->id << ")" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    std::cout << "[GoalReached]: Distance " << distance << " is outside threshold " << threshold
              << " (goal_lanelet_id=" << goal_lanelet->id << ")" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour
#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_REACHED_CONDITION_HPP_
