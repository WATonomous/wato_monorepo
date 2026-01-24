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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "behaviour/behaviour_tree.hpp"
#include "behaviour/dynamic_object_store.hpp"
#include "behaviour/interfaces/interface_base.hpp"

namespace behaviour
{
/**
   * @class BehaviourNode
   * @brief A lifecycle node that manages Eve's behavior tree.
   *
   * TODO(wato): Modify the behaviortree_ros2 library to allow BT::RosNodeParams::nh
   * to accept a rclcpp_lifecycle::LifecycleNode::SharedPtr directly, removing the
   * need for the secondhand 'ros_node_' workaround.
   *
   * Lifecycle order:
   *   on_configure: Create TF, ros_node_, BehaviourTree, then interfaces
   *   on_activate: Activate all interfaces (subscribers start, timers start)
   *   on_deactivate: Deactivate all interfaces
   *   on_cleanup: Clear interfaces and reset tree
   */
class BehaviourNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BehaviourNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BehaviourNode() override;

  rclcpp::Node::SharedPtr get_ros_node() const
  {
    return ros_node_;
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void createInterfaces();

  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_;
  std::string base_frame_;

  std::shared_ptr<BehaviourTree> tree_;
  std::shared_ptr<DynamicObjectStore> dynamic_object_store_;
  std::vector<std::unique_ptr<interfaces::InterfaceBase>> interfaces_;
};
}  // namespace behaviour

#endif  // BEHAVIOUR__BEHAVIOUR_NODE_HPP_
