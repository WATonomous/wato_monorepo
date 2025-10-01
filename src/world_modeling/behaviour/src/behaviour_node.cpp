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

#include "behaviour/behaviour_node.hpp"

#include <chrono>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviour/nodes/actions/execute_behaviour_action.hpp"
#include "behaviour/nodes/decorators/rate_controller.hpp"

// This node is not currently used in a lifecycle context.
// It is implemented as a standard rclcpp::Node if ever the workaround in the lc_node is a problem (see line 81).

BehaviourNode::BehaviourNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("behaviour_node", options)
{
  this->declare_parameter("bt_tree_file", "main_tree.xml");
  this->declare_parameter("rate_hz", 10.0);

  RCLCPP_INFO(this->get_logger(), "BehaviourNode constructed.");
}

BehaviourNode::~BehaviourNode()
{
  RCLCPP_INFO(this->get_logger(), "Stopping Timer");
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Resetting blackboard and timer");
  blackboard_.reset();
  timer_.reset();
}

void BehaviourNode::setup_tree()
{
  RCLCPP_INFO(this->get_logger(), "Setting up Behavior Tree...");

  // Needed for ros wrapped bt nodes
  BT::RosNodeParams params;
  params.nh = this->shared_from_this();

  // Register Nodes
  factory_.registerNodeType<behaviour::ExecuteBehaviourAction>("ExecuteBehaviour", params);
  factory_.registerNodeType<behaviour::RateController>("RateController");

  // Setup Blackboard
  blackboard_ = BT::Blackboard::create();
  blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());

  // Load Tree
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("behaviour");
  std::string bt_xml_name = this->get_parameter("bt_tree_file").as_string();
  std::filesystem::path tree_path = std::filesystem::path(package_share_directory) / "trees" / bt_xml_name;

  tree_ = factory_.createTreeFromFile(tree_path.string(), blackboard_);
  RCLCPP_INFO(this->get_logger(), "BT loaded successfully.");

  // 5. Start the Tick Timer
  double hz = this->get_parameter("rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / hz);

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&BehaviourNode::timer_callback, this));
}

void BehaviourNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Ticking the tree");
  tree_.tickOnce();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BehaviourNode>();

  // Call your setup
  node->setup_tree();

  // Spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
