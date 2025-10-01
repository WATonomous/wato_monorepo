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

#ifndef BEHAVIOUR__BEHAVIOUR_NODE_HPP_
#define BEHAVIOUR__BEHAVIOUR_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "world_modeling_msgs/msg/current_lane_context.hpp"

/**
 * @class BehaviourNode
 * @brief A standard ROS 2 node that manages Eve's behavior tree.
 * The tree immediately upon construction.
 */
class BehaviourNode : public rclcpp::Node
{
public:
  explicit BehaviourNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BehaviourNode() override;

  /**
   * @brief Initializes the Behavior Tree.
   * Must be called after the node is created as a shared_ptr but before spinning.
   */
  void setup_tree();

  BT::Tree & get_tree()
  {
    return tree_;
  }

private:
  /**
   * @brief Main tick function called by the wall timer.
   */
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;
};

#endif  // BEHAVIOUR__BEHAVIOUR_NODE_HPP_
