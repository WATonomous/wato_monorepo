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

#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "attribute_assigner/attribute_assigner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<wato::perception::attribute_assigner::AttributeAssignerNode>(rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());

  auto configured_state = node->configure();
  if (configured_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    auto activated_state = node->activate();
    if (activated_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(node->get_logger(), "Node configured and activated, spinning...");
      exec.spin();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to activate node, current state: %s", activated_state.label().c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure node, current state: %s", configured_state.label().c_str());
  }

  rclcpp::shutdown();
  return 0;
}
