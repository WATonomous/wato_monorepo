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
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/**
 * @class Behaviour Node
 * @brief A lifecycle node that manages eve's behavior tree.
 */
class BehaviourNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BehaviourNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~BehaviourNode() override;

  rclcpp::Node::SharedPtr get_ros_node() const
  {
    return ros_node_;
  }

  // Lifecycle callbacks
protected:
  /**
   * @brief Configures BehaviourNode
   *
   * Initializes blackboard and builds behavior tree
   *
   * @param state reference to LifeCycle node State
   *
   * @return SUCCESS or FAILIURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  /**
   * @brief Activates BehaviourNode and starts ticking the Behavior Tree.
   *
   * Triggers a wall timer to initiate behavior tree ticks at a regular interval
   * defined by the tick rate.
   *
   * @param state Reference to Lifecycle node State.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Deactivates BehaviourNode and stops ticking the Behavior Tree.
   *
   * Cancels and resets the wall timer to stop the regular execution of
   * behavior tree ticks.
   *
   * @param state Reference to Lifecycle node State.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Cleans up and resets member variables.
   *
   * Transitions the node back to the unconfigured state. All configurations
   * are removed, and the behavior engine is reset.
   *
   * @param state Reference to Lifecycle node State.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Called to finalize node shutdown.
   *
   * Ensures all resources are released and the background threads are
   * joined before the node is destroyed.
   *
   * @param state Reference to Lifecycle node State.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  /**
   * @brief Registers all BT nodes in the factory.
   */
  void register_nodes();

  void timer_callback();
  void current_lane_context_callback(const world_modeling_msgs::msg::CurrentLaneContext::SharedPtr msg);

  // ros variables

  rclcpp::Subscription<world_modeling_msgs::msg::CurrentLaneContext>::SharedPtr current_lane_context_sub_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::TimerBase::SharedPtr timer_;

  // behaviour variables

  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;
};

#endif // BEHAVIOUR__BEHAVIOUR_NODE_HPP_
