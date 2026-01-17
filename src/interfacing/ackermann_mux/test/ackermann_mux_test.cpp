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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "ackermann_mux/ackermann_mux_node.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "Ackermann Mux Operation", "[ackermann_mux]")
{
  // System Under Test (SUT)
  rclcpp::NodeOptions options;
  // Define inputs via parameters
  options.parameter_overrides({
    {"inputs.input1.topic", "/input1"},
    {"inputs.input1.priority", 10},
    {"inputs.input1.safety_gating", true},
    {"inputs.input2.topic", "/input2"},
    {"inputs.input2.priority", 20},
    {"inputs.input2.has_mask", true},
    {"inputs.input2.mask_topic", "/input2_mask"},
    {"safety_threshold", 0.5},
    {"publish_rate_hz", 10.0}  // Fast enough for tests
  });

  auto mux_node = std::make_shared<ackermann_mux::AckermannMuxNode>(options);
  add_node(mux_node);

  // Test Inputs
  auto input1_pub = std::make_shared<PublisherTestNode<AckermannDriveStamped>>("/input1", "input1_pub");
  auto input2_pub = std::make_shared<PublisherTestNode<AckermannDriveStamped>>("/input2", "input2_pub");
  add_node(input1_pub);
  add_node(input2_pub);

  // Test Output
  auto output_sub = std::make_shared<SubscriberTestNode<AckermannDriveStamped>>("/ackermann", "output_sub");
  add_node(output_sub);

  // Mask Publisher
  auto mask_pub = std::make_shared<PublisherTestNode<std_msgs::msg::Bool>>("/input2_mask", "mask_pub");
  add_node(mask_pub);

  // Start execution
  start_spinning();

  SECTION("Basic Priority Selection")
  {
    // Publish to low priority input
    AckermannDriveStamped cmd1;
    cmd1.drive.speed = 1.0;
    input1_pub->publish(cmd1);

    // Wait for output (mux runs at 10Hz)
    // We expect the mux to pick up this single input
    auto msg1 = output_sub->expect_next_message().get();
    REQUIRE(msg1.drive.speed == Catch::Approx(1.0));

    // Publish to high priority input
    AckermannDriveStamped cmd2;
    cmd2.drive.speed = 2.0;
    input2_pub->publish(cmd2);

    // Also refresh low priority to ensure both are "active"
    input1_pub->publish(cmd1);

    // Expect high priority wins
    auto msg2 = output_sub->expect_next_message().get();
    REQUIRE(msg2.drive.speed == Catch::Approx(2.0));
  }

  SECTION("Safety Timeout")
  {
    // Publish once
    AckermannDriveStamped cmd1;
    cmd1.drive.speed = 1.0;
    input1_pub->publish(cmd1);

    auto msg1 = output_sub->expect_next_message().get();
    REQUIRE(msg1.drive.speed == Catch::Approx(1.0));

    // Wait longer than safety_threshold (0.5s)
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Because we waited, the inputs should have timed out.
    // The mux should publish emergency (default 0 speed)
    auto msg2 = output_sub->expect_next_message().get();
    REQUIRE(msg2.drive.speed == Catch::Approx(0.0));
  }

  SECTION("No Active Inputs")
  {
    // Don't publish anything
    // Mux should default to emergency (speed 0.0) immediately/eventually
    auto msg = output_sub->expect_next_message().get();
    REQUIRE(msg.drive.speed == Catch::Approx(0.0));
  }

  SECTION("Priority Masking (Idle)")
  {
    // Publish to low priority input
    AckermannDriveStamped cmd1;
    cmd1.drive.speed = 1.0;
    input1_pub->publish(cmd1);

    // Publish to high priority input
    AckermannDriveStamped cmd2;
    cmd2.drive.speed = 2.0;
    input2_pub->publish(cmd2);

    // Verify high priority wins initially
    auto msg1 = output_sub->expect_next_message().get();
    REQUIRE(msg1.drive.speed == Catch::Approx(2.0));

    // Refresh inputs to keep them alive
    input1_pub->publish(cmd1);
    input2_pub->publish(cmd2);

    // MASK the high priority input (simulate idle)
    std_msgs::msg::Bool mask_msg;
    mask_msg.data = true;
    mask_pub->publish(mask_msg);

    // Now low priority should win because high priority is masked
    auto msg2 = output_sub->expect_next_message().get();
    REQUIRE(msg2.drive.speed == Catch::Approx(1.0));

    // UNMASK
    mask_msg.data = false;
    mask_pub->publish(mask_msg);

    // Refresh inputs
    input1_pub->publish(cmd1);
    input2_pub->publish(cmd2);

    // High priority should win again
    auto msg3 = output_sub->expect_next_message().get();
    REQUIRE(msg3.drive.speed == Catch::Approx(2.0));
  }
}
