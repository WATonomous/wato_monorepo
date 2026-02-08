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

#ifndef BEHAVIOUR__BEHAVIOUR_TREE_HPP_
#define BEHAVIOUR__BEHAVIOUR_TREE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace behaviour
{
/**
 * @class BehaviourTree
 * @brief Manages the BT factory, blackboard, and execution logic.
 */
class BehaviourTree
{
public:
  /**
   * @brief Initializes the tree, registers nodes, and loads the XML file.
   */
  BehaviourTree(rclcpp::Node::SharedPtr node, const std::string & tree_file_path, bool logging = false);

  /** @brief Performs a single tick of the behavior tree. */
  void tick();

  /** @brief Sets a value on the blackboard. */
  template <typename T>
  void updateBlackboard(const std::string & key, const T & value)
  {
    blackboard_->set<T>(key, value);
  }

  /** @brief Gets a value from the blackboard. */
  template <typename T>
  T getBlackboard(const std::string & key) const
  {
    return blackboard_->get<T>(key);
  }

private:
  /** @brief Registers custom BT nodes with the factory. */
  void registerNodes();

  /** @brief Loads the XML file and creates the internal tree. */
  void buildTree(const std::string & tree_file_path);

  std::unique_ptr<BT::StdCoutLogger> cout_logger_;

  rclcpp::Node::SharedPtr node_;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_TREE_HPP_
