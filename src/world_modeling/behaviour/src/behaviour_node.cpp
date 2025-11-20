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
#include "behaviour/bt_node_utils.hpp"

BehaviourNode::BehaviourNode() : Node("behaviour_node") {
  RCLCPP_INFO(this->get_logger(), "BehaviourNode has been started.");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // create a single shared node instance that BT leaf nodes will use
  auto node = std::make_shared<BehaviourNode>();
  wato::world_modeling::behaviour::set_shared_node(node);

  // create BT factory and load tree XML from package share
  BT::BehaviorTreeFactory factory;

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("behaviour");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Could not find package share directory: %s", e.what());
    return 1;
  }

  std::string xml_path = pkg_share + "/bt/xml/main_tree.xml";
  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_path);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create BT from file: %s", ex.what());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "BehaviourNode initialized (shared node set)");

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
