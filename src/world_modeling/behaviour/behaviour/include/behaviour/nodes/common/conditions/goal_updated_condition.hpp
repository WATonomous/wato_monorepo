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

#ifndef BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_UPDATED_CONDITION_HPP_
#define BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_UPDATED_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace behaviour
{
/**
 * @class GoalUpdatedCondition
 * @brief Returns SUCCESS when the current goal point differs from the previously seen goal point.
 */
class GoalUpdatedCondition : public BT::ConditionNode, protected BTLoggerBase
{
public:
  GoalUpdatedCondition(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::ConditionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PointStamped::SharedPtr>("goal_point")};
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_DEBUG_STREAM(logger(), "Missing " << port_name << " input");
    };

    auto goal_point = ports::tryGetPtr<geometry_msgs::msg::PointStamped>(*this, "goal_point");
    if (!ports::require(goal_point, "goal_point", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    if (!has_previous_goal_ || *goal_point != previous_goal_) {
      previous_goal_ = *goal_point;
      has_previous_goal_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  bool has_previous_goal_ = false;
  geometry_msgs::msg::PointStamped previous_goal_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__CONDITIONS__GOAL_UPDATED_CONDITION_HPP_
