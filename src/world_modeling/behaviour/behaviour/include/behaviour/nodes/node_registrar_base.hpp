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

#ifndef BEHAVIOUR__NODES__NODE_REGISTRAR_BASE_HPP_
#define BEHAVIOUR__NODES__NODE_REGISTRAR_BASE_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>

/**
 * @class NodeRegistrarBase
 * @brief Base class for registering behaviour tree nodes.
 */
class NodeRegistrarBase
{
public:
  explicit NodeRegistrarBase(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node ? node->get_logger() : rclcpp::get_logger("node_registrar"))
  {
    if (!node_) {
      throw std::runtime_error("Registrar constructed with null ROS node");
    }
  }

  virtual void register_nodes(BT::BehaviorTreeFactory &, const BT::RosNodeParams &) = 0;
  ~NodeRegistrarBase() = default;

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
};

#endif  // BEHAVIOUR__NODES__NODE_REGISTRAR_BASE_HPP_
