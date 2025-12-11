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

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wato_test/wato_test.hpp>

#include "patchworkpp/ground_removal_node.hpp"

using wato::perception::patchworkpp::GroundRemovalNode;

// Disable ROS 2 colorized output to prevent ANSI escape codes in JUnit XML
namespace
{
struct DisableROSColorOutput
{
  DisableROSColorOutput()
  {
    // Set RCUTILS_COLORIZED_OUTPUT=0 before ROS 2 initialization
    // This prevents ANSI escape codes from being written to test output
    setenv("RCUTILS_COLORIZED_OUTPUT", "0", 1);
  }
};
// Global instance ensures this runs before any ROS initialization
static DisableROSColorOutput disable_color_output;
}  // namespace

// Helper to wait for a message with timeout using polling
template<typename MessageType>
std::optional<MessageType> wait_for_message(
  std::shared_ptr<wato::test::SubscriberTestNode<MessageType>> subscriber,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
{
  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto msg = subscriber->get_last_message();
    if (msg.has_value()) {
      return msg;
    }
    if (std::chrono::steady_clock::now() - start > timeout) {
      return std::nullopt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// Helper to create a simple PointCloud2 for testing
sensor_msgs::msg::PointCloud2 createTestPointCloud(size_t num_points)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(num_points);
  msg.is_bigendian = false;
  msg.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(num_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  // Create points at proper range (beyond min_range ~1.0m) for Patchwork++
  // Generate points in a grid pattern from 3m to 15m range
  const float min_range = 3.0f;
  const float max_range = 15.0f;
  const float sensor_height = 1.88f;  // Match default parameter value
  
  for (size_t i = 0; i < num_points; ++i) {
    // Distribute points in a grid pattern
    size_t grid_size = static_cast<size_t>(std::sqrt(static_cast<float>(num_points)));
    if (grid_size == 0) grid_size = 1;
    size_t row = i / grid_size;
    size_t col = i % grid_size;
    
    float x = min_range + (max_range - min_range) * static_cast<float>(row) / static_cast<float>(grid_size);
    float y = -5.0f + 10.0f * static_cast<float>(col) / static_cast<float>(grid_size);
    float z = -sensor_height;  // Ground plane at sensor height below origin
    
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  msg.header.frame_id = "test_frame";
  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}

// ============================================================================
// TEST CASES
// ============================================================================

// ----------------------------------------------------------------------------
// Integration Tests - Lifecycle and Topics
// ----------------------------------------------------------------------------
TEST_CASE_METHOD(wato::test::TestExecutorFixture, "ROS Integration Tests", "[ros_integration]")
{
  // Add timeout protection
  auto test_timeout = std::chrono::seconds(30);
  auto test_start = std::chrono::steady_clock::now();

  rclcpp::NodeOptions options;
  auto node = std::make_shared<GroundRemovalNode>(options);

  // Check if node creation succeeded
  REQUIRE(node != nullptr);

  add_node(node);
  start_spinning();

  // Add a small delay to ensure executor is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  SECTION("Topic name constants")
  {
    REQUIRE(std::string(GroundRemovalNode::kCloudTopic) == "input_cloud");
    REQUIRE(std::string(GroundRemovalNode::kGroundTopic) == "ground_cloud");
    REQUIRE(std::string(GroundRemovalNode::kNonGroundTopic) == "non_ground_cloud");
  }

  SECTION("Lifecycle state transitions")
  {
    // Add timeout check
    if (std::chrono::steady_clock::now() - test_start > test_timeout) {
      FAIL("Test timeout exceeded");
    }

    // Initially unconfigured
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    // Configure - returns State, check that it's INACTIVE
    auto configured_state = node->configure();
    REQUIRE(configured_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    // Activate - returns State, check that it's ACTIVE
    auto activated_state = node->activate();
    REQUIRE(activated_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // Deactivate - returns State, check that it's INACTIVE
    auto deactivated_state = node->deactivate();
    REQUIRE(deactivated_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  // Add explicit cleanup at end
  if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    node->deactivate();
  }
  if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    node->cleanup();
  }
}

// ----------------------------------------------------------------------------
// QoS Configuration Tests - Isolated (No Executor)
// ----------------------------------------------------------------------------
TEST_CASE("QoS Configuration Tests - Isolated", "[qos]")
{
  // Don't use TestExecutorFixture - these tests don't need spinning
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  SECTION("Subscriber QoS - reliable")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_subscriber_reliability", "reliable"));
    node->set_parameter(rclcpp::Parameter("qos_subscriber_depth", 10));

    auto state = node->configure();
    REQUIRE(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Subscriber QoS - best_effort")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_subscriber_reliability", "best_effort"));

    auto state = node->configure();
    REQUIRE(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Subscriber QoS - invalid (should default)")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_subscriber_reliability", "invalid_policy"));

    // Should still configure successfully with default
    REQUIRE_NOTHROW(node->configure());
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
  }

  SECTION("Publisher QoS - reliable + transient_local")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_publisher_reliability", "reliable"));
    node->set_parameter(rclcpp::Parameter("qos_publisher_durability", "transient_local"));
    node->set_parameter(rclcpp::Parameter("qos_publisher_depth", 10));

    auto state = node->configure();
    REQUIRE(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
  }

  SECTION("Publisher QoS - best_effort + volatile")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_publisher_reliability", "best_effort"));
    node->set_parameter(rclcpp::Parameter("qos_publisher_durability", "volatile"));

    auto state = node->configure();
    REQUIRE(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
  }

  SECTION("Publisher QoS - invalid reliability (should default)")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_publisher_reliability", "invalid_reliability"));

    REQUIRE_NOTHROW(node->configure());
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
  }

  SECTION("Publisher QoS - invalid durability (should default)")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    node->set_parameter(rclcpp::Parameter("qos_publisher_durability", "invalid_durability"));

    REQUIRE_NOTHROW(node->configure());
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// ----------------------------------------------------------------------------
// Message Handling Tests
// ----------------------------------------------------------------------------
TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Message Handling Tests", "[messages]")
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<GroundRemovalNode>(options);
  REQUIRE(node != nullptr);

  add_node(node);
  start_spinning();

  SECTION("removeGround callback handles empty point cloud")
  {
    node->configure();
    node->activate();

    auto input_pub = std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(
      GroundRemovalNode::kCloudTopic);
    add_node(input_pub);

    // Give time for connections
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Send empty point cloud
    auto empty_cloud = createTestPointCloud(0);
    input_pub->publish(empty_cloud);

    // Short wait - don't expect output
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Just verify it didn't crash
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    node->deactivate();
  }

  SECTION("removeGround callback handles invalid point cloud gracefully")
  {
    node->configure();
    node->activate();

    auto input_pub = std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(
      GroundRemovalNode::kCloudTopic);
    add_node(input_pub);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Send point cloud with missing fields
    sensor_msgs::msg::PointCloud2 invalid_msg;
    invalid_msg.width = 10;
    invalid_msg.height = 1;
    invalid_msg.header.frame_id = "test";
    invalid_msg.header.stamp = rclcpp::Clock().now();
    // No fields defined - should trigger error

    input_pub->publish(invalid_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Verify it didn't crash
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    node->deactivate();
  }

  // Cleanup
  if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      node->deactivate();
    }
    node->cleanup();
  }
}

// ----------------------------------------------------------------------------
// Message Flow Integration Test
// ----------------------------------------------------------------------------
TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Simple Message Flow Test", "[simple]")
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<GroundRemovalNode>(options);
  REQUIRE(node != nullptr);

  add_node(node);
  start_spinning();

  node->configure();
  node->activate();

  // Create subscribers to verify output
  auto ground_sub = std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(
    GroundRemovalNode::kGroundTopic);
  auto nonground_sub = std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(
    GroundRemovalNode::kNonGroundTopic);
  add_node(ground_sub);
  add_node(nonground_sub);

  // Create publisher to send input
  auto input_pub = std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(
    GroundRemovalNode::kCloudTopic);
  add_node(input_pub);

  // Wait for connection
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Send a test point cloud
  auto test_cloud = createTestPointCloud(100);
  input_pub->publish(test_cloud);

  // Wait for ground cloud message with timeout - make requirement explicit
  auto ground_msg = wait_for_message(ground_sub, std::chrono::seconds(2));
  REQUIRE(ground_msg.has_value());
  REQUIRE(ground_msg->header.frame_id == test_cloud.header.frame_id);

  // Wait for non-ground cloud message with timeout - make requirement explicit
  auto nonground_msg = wait_for_message(nonground_sub, std::chrono::seconds(2));
  REQUIRE(nonground_msg.has_value());
  REQUIRE(nonground_msg->header.frame_id == test_cloud.header.frame_id);

  // Verify node is still active
  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node->deactivate();
  node->cleanup();
}

// ----------------------------------------------------------------------------
// Statistics and Diagnostics Tests
// ----------------------------------------------------------------------------
TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Statistics and Diagnostics Tests", "[statistics]")
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<GroundRemovalNode>(options);
  add_node(node);
  start_spinning();

  node->configure();
  node->activate();

  SECTION("Statistics update after processing")
  {
    auto input_pub = std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(
      GroundRemovalNode::kCloudTopic);
    add_node(input_pub);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send multiple point clouds
    for (int i = 0; i < 3; ++i) {
      auto test_cloud = createTestPointCloud(100);
      input_pub->publish(test_cloud);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Statistics should have been updated (we can't check private members directly)
    // But we can verify the node is still functioning
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  node->deactivate();
  node->cleanup();
}

// ----------------------------------------------------------------------------
// Parameter Tests (No Executor)
// ----------------------------------------------------------------------------
TEST_CASE("Parameter Handling Tests", "[parameters]")
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  SECTION("Parameter declaration and retrieval")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    REQUIRE(node->has_parameter("sensor_height"));
    REQUIRE(node->has_parameter("num_iter"));
    REQUIRE(node->has_parameter("num_lpr"));
    REQUIRE(node->has_parameter("num_min_pts"));
    REQUIRE(node->has_parameter("th_seeds"));
    REQUIRE(node->has_parameter("th_dist"));
    REQUIRE(node->has_parameter("th_seeds_v"));
    REQUIRE(node->has_parameter("th_dist_v"));
    REQUIRE(node->has_parameter("max_range"));
    REQUIRE(node->has_parameter("min_range"));
    REQUIRE(node->has_parameter("uprightness_thr"));
    REQUIRE(node->has_parameter("enable_RNR"));
    REQUIRE(node->has_parameter("verbose"));
    REQUIRE(node->has_parameter("qos_subscriber_reliability"));
    REQUIRE(node->has_parameter("qos_subscriber_depth"));
    REQUIRE(node->has_parameter("qos_publisher_reliability"));
    REQUIRE(node->has_parameter("qos_publisher_durability"));
    REQUIRE(node->has_parameter("qos_publisher_depth"));
  }

  SECTION("Parameter default values")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    REQUIRE(node->get_parameter("sensor_height").as_double() == 1.88);  // FIXED
    REQUIRE(node->get_parameter("num_iter").as_int() == 3);
    REQUIRE(node->get_parameter("num_lpr").as_int() == 20);
    REQUIRE(node->get_parameter("num_min_pts").as_int() == 0);
    REQUIRE(node->get_parameter("th_seeds").as_double() == 0.3);
    REQUIRE(node->get_parameter("th_dist").as_double() == 0.10);
    REQUIRE(node->get_parameter("th_seeds_v").as_double() == 0.25);
    REQUIRE(node->get_parameter("th_dist_v").as_double() == 0.85);
    REQUIRE(node->get_parameter("max_range").as_double() == 80.0);
    REQUIRE(node->get_parameter("min_range").as_double() == 1.0);
    REQUIRE(node->get_parameter("uprightness_thr").as_double() == 0.101);
    REQUIRE(node->get_parameter("enable_RNR").as_bool() == false);
    REQUIRE(node->get_parameter("verbose").as_bool() == true);
    REQUIRE(node->get_parameter("qos_subscriber_reliability").as_string() == "best_effort");
    REQUIRE(node->get_parameter("qos_subscriber_depth").as_int() == 10);
    REQUIRE(node->get_parameter("qos_publisher_reliability").as_string() == "reliable");
    REQUIRE(node->get_parameter("qos_publisher_durability").as_string() == "transient_local");
    REQUIRE(node->get_parameter("qos_publisher_depth").as_int() == 10);
  }

  SECTION("Parameter modification and reconfiguration")
  {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GroundRemovalNode>(options);

    // Modify parameters before configuration
    node->set_parameter(rclcpp::Parameter("sensor_height", 2.0));
    node->set_parameter(rclcpp::Parameter("num_iter", 5));
    node->set_parameter(rclcpp::Parameter("verbose", false));
    // Configure with new values
    node->configure();

    // Verify new values are used
    REQUIRE(node->get_parameter("sensor_height").as_double() == 2.0);
    REQUIRE(node->get_parameter("num_iter").as_int() == 5);
    REQUIRE(node->get_parameter("verbose").as_bool() == false);

    node->cleanup();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

