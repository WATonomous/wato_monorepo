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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviour/dynamic_object_store.hpp"
#include "world_model_msgs/msg/dynamic_object.hpp"

namespace behaviour
{

class BehaviourTree
{
public:
  /**
     * @brief Construct and initialize the behavior tree.
     * @param lifecycle_node The lifecycle node (for logging and parameters)
     * @param ros_node The ROS node used by BT ROS action/service nodes
     * @param tree_file_path Path to the behavior tree XML file
     * @param store Pointer to the dynamic object store
     */
  BehaviourTree(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node,
    rclcpp::Node::SharedPtr ros_node,
    const std::string & tree_file_path,
    std::shared_ptr<DynamicObjectStore> store);

  /**
     * @brief Tick the tree once.
     */
  void tick();

  /**
     * @brief Update a blackboard entry.
     * @tparam T The type of the value
     * @param key The blackboard key
     * @param value The value to set
     */
  template <typename T>
  void updateBlackboard(const std::string & key, const T & value)
  {
    blackboard_->set<T>(key, value);
  }

  /**
     * @brief Get a value from the blackboard.
     * @tparam T The type of the value
     * @param key The blackboard key
     * @return The value, or throws if not found
     */
  template <typename T>
  T getBlackboard(const std::string & key) const
  {
    return blackboard_->get<T>(key);
  }

private:
  void registerNodes();
  void buildTree(const std::string & tree_file_path);

  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<DynamicObjectStore> store_;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_TREE_HPP_
