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
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <wato_test/wato_test.hpp>

#include "bevfusion/bevfusion_node.hpp"

using wato::perception::bevfusion::BEVFusionNode;

// =============================================================================
// TEST 1: Lifecycle state machine
// WHY: BEVFusionNode is a lifecycle node — it must transition cleanly through
//      all standard states. Verifies the skeleton wires up ROS lifecycle correctly.
// =============================================================================
TEST_CASE("Lifecycle node transitions through all states", "[node][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  REQUIRE(node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  rclcpp::shutdown();
}

// =============================================================================
// TEST 2: Parameter overrides via NodeOptions are respected
// WHY: The node must accept runtime overrides (e.g., from YAML via launch file)
//      so teams can swap model directories or tune thresholds without recompiling.
// =============================================================================
TEST_CASE("declareParameters: NodeOptions overrides are respected", "[params][fast]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.append_parameter_override("model_dir", "/custom/test/bevfusion/model");
  options.append_parameter_override("confidence_threshold", 0.75);

  auto node = std::make_shared<BEVFusionNode>(options);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  REQUIRE(node->get_parameter("model_dir").as_string() == "/custom/test/bevfusion/model");
  REQUIRE_THAT(node->get_parameter("confidence_threshold").as_double(), Catch::Matchers::WithinRel(0.75));

  rclcpp::shutdown();
}
