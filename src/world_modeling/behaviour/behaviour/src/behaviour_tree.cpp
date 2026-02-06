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

#include "behaviour/behaviour_tree.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "behaviour/nodes/common/registrar.hpp"
#include "behaviour/nodes/intersection/registrar.hpp"
#include "behaviour/nodes/lane_navigation/registrar.hpp"

namespace behaviour
{

  BehaviourTree::BehaviourTree(rclcpp::Node::SharedPtr node, const std::string &tree_file_path, bool logging)
      : node_(std::move(node))
  {
    registerNodes();
    buildTree(tree_file_path);

    if (logging)
    {
      // This logger prints state transitions [IDLE -> RUNNING -> SUCCESS] to terminal (configured in config)
      cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    }
  }

  void BehaviourTree::registerNodes()
  {
    if (!node_)
    {
      throw std::runtime_error("ROS node provided to BehaviourTree is null!");
    }

    // Default params for most ROS-linked BT nodes
    BT::RosNodeParams params;
    params.nh = node_;

    // load all the node registrars
    CommonNodeRegistrar common_registrar;
    IntersectionNodeRegistrar intersection_registrar;
    LaneNavigationNodeRegistrar lane_navigation_registrar;

    common_registrar.register_nodes(factory_, params);
    intersection_registrar.register_nodes(factory_, params);
    lane_navigation_registrar.register_nodes(factory_, params);
  }

  void BehaviourTree::buildTree(const std::string &tree_file_path)
  {
    blackboard_ = BT::Blackboard::create();

    // Set the node on the blackboard so custom nodes can access it
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);

    RCLCPP_INFO(node_->get_logger(), "Loading Behavior Tree from: %s", tree_file_path.c_str());
    tree_ = factory_.createTreeFromFile(tree_file_path, blackboard_);
  }

  void BehaviourTree::tick()
  {
    tree_.tickOnce();
  }

} // namespace behaviour
