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

#include <behaviortree_ros2/bt_action_node.hpp>

// type
#include "behaviour/types/lattice_planner_behaviour.hpp"
#include "world_modeling_msgs/action/execute_behaviour.hpp"

namespace behaviour
{
  using ExecuteBehaviour = world_modeling_msgs::action::ExecuteBehaviour;

  /**
   * @class ExecuteBehaviourAction
   * @brief BT node to command the vehicle to execute a specific behavior (lane follow, lane change, etc.)
   */
  class ExecuteBehaviourAction : public BT::RosActionNode<ExecuteBehaviour>
  {
  public:
    ExecuteBehaviourAction(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
        : BT::RosActionNode<ExecuteBehaviour>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({BT::InputPort<std::string>("behaviour", "The behavior to execute")});
    }

    bool setGoal(RosActionNode::Goal &goal) override
    {
      auto behaviour = getInput<LatticePlannerBehaviour>("behaviour");
      if (!behaviour)
      {
        return false;
      }
      goal.behaviour = behaviour.value();
      return true;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
    {
      (void)feedback;
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
      (void)error;
      return BT::NodeStatus::FAILURE;
    }
  };
} // namespace behaviour

#endif // BEHAVIOUR__EXECUTE_BEHAVIOUR_ACTION_HPP_
