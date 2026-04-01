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

#ifndef BEHAVIOUR__NODES__COMMON__ACTIONS__GET_GOAL_POINT_SUBSCRIBER_HPP_
#define BEHAVIOUR__NODES__COMMON__ACTIONS__GET_GOAL_POINT_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"

namespace behaviour
{
/**
 * @class GetGoalPointSubscriber
 * @brief RosTopicSubNode that forwards the latest goal-point message to the blackboard.
 */
class GetGoalPointSubscriber : public BT::RosTopicSubNode<geometry_msgs::msg::PointStamped>
{
public:
  GetGoalPointSubscriber(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<geometry_msgs::msg::PointStamped>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<geometry_msgs::msg::PointStamped::SharedPtr>("goal_point"),
      BT::OutputPort<std::string>("error_message"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PointStamped> & last_msg) override
  {
    if (!last_msg) {
      setOutput("error_message", "no_message_received");
      return BT::NodeStatus::FAILURE;
    }

    setOutput("goal_point", last_msg);
    return BT::NodeStatus::SUCCESS;
  }

  bool latchLastMessage() const override
  {
    return true;
  }
};
}  // namespace behaviour

#endif  // BEHAVIOUR__NODES__COMMON__ACTIONS__GET_GOAL_POINT_SUBSCRIBER_HPP_
