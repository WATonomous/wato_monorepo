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

  // keep behaviour node alive; BT factory and nodes will be added later
  RCLCPP_INFO(node->get_logger(), "BehaviourNode initialized (shared node set)");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
