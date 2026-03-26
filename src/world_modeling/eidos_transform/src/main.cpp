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

#include <rclcpp/rclcpp.hpp>

#include "eidos_transform/eidos_transform_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<eidos_transform::EidosTransformNode>(options);
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(rclcpp::get_logger("eidos_transform"), "EidosTransform node started");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
