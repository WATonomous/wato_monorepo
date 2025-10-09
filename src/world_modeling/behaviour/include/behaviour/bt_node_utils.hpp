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

#ifndef BEHAVIOUR__BT_NODE_UTILS_HPP_
#define BEHAVIOUR__BT_NODE_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace wato::world_modeling::behaviour {

// Set a shared rclcpp::Node instance that BT leaf nodes can access.
// Call set_shared_node() once (from main) before BT nodes are created.
void set_shared_node(const rclcpp::Node::SharedPtr &node);

// Get the shared rclcpp::Node previously set. Returns nullptr if not set.
rclcpp::Node::SharedPtr get_shared_node();

} // namespace wato::world_modeling::behaviour

#endif // BEHAVIOUR__BT_NODE_UTILS_HPP_
