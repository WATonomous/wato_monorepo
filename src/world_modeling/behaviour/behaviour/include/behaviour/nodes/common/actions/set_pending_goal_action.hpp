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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__SET_PENDING_GOAL_ACTION_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__SET_PENDING_GOAL_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

#include "behaviour/nodes/bt_logger_base.hpp"
#include "behaviour/utils/ports.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace behaviour
{
/**
 * @class SetPendingGoalAction
 * @brief Moves the requested goal into the pending goal slot and clears the requested slot.
 */
class SetPendingGoalAction : public BT::SyncActionNode, protected BTLoggerBase
{
public:
  SetPendingGoalAction(const std::string & name, const BT::NodeConfig & config, const rclcpp::Logger & logger)
  : BT::SyncActionNode(name, config)
  , BTLoggerBase(logger)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PointStamped::SharedPtr>("requested_goal_point"),
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("out_pending_goal_point"),
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("out_requested_goal_point"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto missing_input_callback = [&](const char * port_name) {
      RCLCPP_ERROR(logger(), "Missing input port: %s", port_name);
    };

    auto requested_goal_point = ports::tryGetPtr<geometry_msgs::msg::PointStamped>(*this, "requested_goal_point");
    if (!ports::require(requested_goal_point, "requested_goal_point", missing_input_callback)) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PointStamped::SharedPtr cleared_requested_goal_point = nullptr;
    setOutput("out_pending_goal_point", requested_goal_point);
    setOutput("out_requested_goal_point", cleared_requested_goal_point);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__SET_PENDING_GOAL_ACTION_HPP_
