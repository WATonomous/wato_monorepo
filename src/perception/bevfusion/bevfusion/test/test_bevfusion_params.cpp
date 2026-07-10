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

#include "bevfusion/bevfusion_node.hpp"

using wato::perception::bevfusion::BEVFusionNode;

namespace
{

struct RclcppGuard
{
  RclcppGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

// =============================================================================
// TEST 1: Lifecycle state machine
// WHY: BEVFusionNode is a lifecycle node — it must transition cleanly through
//      all standard states. Verifies the skeleton wires up ROS lifecycle correctly.
// =============================================================================
TEST_CASE("Lifecycle node transitions through all states", "[node][fast]")
{
  RclcppGuard guard;
  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});

  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  REQUIRE(node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// =============================================================================
// TEST 2: Default parameter values after configure
// WHY: declareParameters() must register sane defaults so the node runs
//      out-of-the-box without a YAML override. Verifies every parameter
//      declared in params.yaml has the expected built-in default.
// =============================================================================
TEST_CASE("declareParameters: default values are set correctly", "[params][fast]")
{
  RclcppGuard guard;
  auto node = std::make_shared<BEVFusionNode>(rclcpp::NodeOptions{});
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Model directory
  REQUIRE(node->get_parameter("model_dir").as_string() == "/opt/watonomous/models/bevfusion/resnet50");

  // Confidence threshold
  REQUIRE_THAT(node->get_parameter("confidence_threshold").as_double(), Catch::Matchers::WithinRel(0.3));

  // Camera names — must match the 6 nuScenes-layout cameras on Eve
  const auto camera_names = node->get_parameter("camera_names").as_string_array();
  REQUIRE(camera_names.size() == 6);
  REQUIRE(camera_names[0] == "camera_pano_nn");
  REQUIRE(camera_names[1] == "camera_pano_ne");
  REQUIRE(camera_names[2] == "camera_pano_nw");
  REQUIRE(camera_names[3] == "camera_pano_ss");
  REQUIRE(camera_names[4] == "camera_pano_se");
  REQUIRE(camera_names[5] == "camera_pano_sw");

  // Topic names
  REQUIRE(node->get_parameter("topic_multi_image").as_string() == "/multi_camera_sync/multi_image_compressed");
  REQUIRE(node->get_parameter("topic_lidar").as_string() == "/lidar/all/points_merged");
  REQUIRE(node->get_parameter("topic_camera_info").as_string() == "/multi_camera_sync/multi_camera_info");

  // QoS — subscriber must default to best_effort to receive Nitros topics
  REQUIRE(node->get_parameter("qos_subscriber_reliability").as_string() == "best_effort");
  REQUIRE(node->get_parameter("qos_publisher_reliability").as_string() == "reliable");
  REQUIRE(node->get_parameter("qos_publisher_durability").as_string() == "transient_local");

  // Synchronization
  REQUIRE(node->get_parameter("sync_queue_size").as_int() == 10);
  REQUIRE_THAT(node->get_parameter("sync_max_time_diff_ms").as_double(), Catch::Matchers::WithinRel(200.0));
}

// =============================================================================
// TEST 3: Parameter overrides via NodeOptions are respected
// WHY: The node must accept runtime overrides (e.g., from YAML via launch file)
//      so teams can swap model directories or tune thresholds without recompiling.
// =============================================================================
TEST_CASE("declareParameters: NodeOptions overrides are respected", "[params][fast]")
{
  RclcppGuard guard;
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_dir", "/custom/test/bevfusion/model");
  options.append_parameter_override("confidence_threshold", 0.75);

  auto node = std::make_shared<BEVFusionNode>(options);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  REQUIRE(node->get_parameter("model_dir").as_string() == "/custom/test/bevfusion/model");
  REQUIRE_THAT(node->get_parameter("confidence_threshold").as_double(), Catch::Matchers::WithinRel(0.75));
}
