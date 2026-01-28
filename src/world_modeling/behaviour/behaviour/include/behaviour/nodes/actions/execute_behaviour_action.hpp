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

#ifndef BEHAVIOUR__EXECUTE_BEHAVIOUR_ACTION_HPP_
#define BEHAVIOUR__EXECUTE_BEHAVIOUR_ACTION_HPP_

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>

#include "behaviour/utils/utils.hpp"
#include "behaviour_msgs/action/execute_behaviour.hpp"

namespace behaviour
{

/**
   * @class ExecuteBehaviourAction
   * @brief BT node to command the vehicle to execute a specific behavior (lane follow, lane change, etc.)
   *
   * TODO(wato): action server on lattice planner
   */
class ExecuteBehaviourAction : public BT::RosActionNode<behaviour_msgs::action::ExecuteBehaviour>
{
public:
  ExecuteBehaviourAction(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosActionNode<behaviour_msgs::action::ExecuteBehaviour>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("behaviour", "The behavior to execute")});
  }

  bool setGoal(RosActionNode::Goal & goal) override
  {
    auto lane_behaviour = ports::get<std::string>(*this, "behaviour");
    RCLCPP_INFO(logger(), "Execute behaviour goal: %s", lane_behaviour.c_str());
    goal.behaviour = lane_behaviour;
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    (void)feedback;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "ExecuteBehaviour action failed: %d", error);
    // sending success for now for testing
    return BT::NodeStatus::SUCCESS;
    // return BT::NodeStatus::FAILURE;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__EXECUTE_BEHAVIOUR_ACTION_HPP_
