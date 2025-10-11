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
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch.hpp>
#include <wato_test/wato_test.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

namespace wato
{

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Multiple nodes with TestExecutorFixture", "[multi_node]")
{
  std::string test_topic = "/multi_node_topic";

  // Create publisher and subscriber test nodes
  auto pub_node =
    std::make_shared<wato::test::PublisherTestNode<std_msgs::msg::String>>(test_topic, "publisher_node");
  auto sub_node =
    std::make_shared<wato::test::SubscriberTestNode<std_msgs::msg::String>>(test_topic, "subscriber_node");

  // Add both nodes to the executor
  add_node(pub_node);
  add_node(sub_node);
  start_spinning();

  SECTION("Both nodes are running")
  {
    REQUIRE(pub_node != nullptr);
    REQUIRE(sub_node != nullptr);
    REQUIRE(pub_node->get_name() == std::string("publisher_node"));
    REQUIRE(sub_node->get_name() == std::string("subscriber_node"));
  }

  SECTION("Communication between nodes")
  {
    // Set up expectation for receiving message
    auto message_future = sub_node->expect_next_message();

    // Publish a message
    std_msgs::msg::String msg;
    msg.data = "test_message";
    pub_node->publish(msg);

    // Wait for and verify the message was received
    auto status = message_future.wait_for(std::chrono::milliseconds(100));
    REQUIRE(status == std::future_status::ready);
    auto received = message_future.get();
    REQUIRE(received.data == "test_message");
  }

  SECTION("Multiple messages are received correctly")
  {
    // Reset message count for clean test
    sub_node->reset_for_new_test();

    // Allow time for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const int num_messages = 3;

    // Publish messages with small delays to ensure proper ordering
    for (int i = 0; i < num_messages; ++i) {
      std_msgs::msg::String msg;
      msg.data = "Message " + std::to_string(i);
      pub_node->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait for all messages to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Verify message count
    REQUIRE(sub_node->get_message_count() == num_messages);

    // Check last message
    auto last_msg = sub_node->get_last_message();
    REQUIRE(last_msg.has_value());
    REQUIRE(last_msg->data == "Message " + std::to_string(num_messages - 1));
  }

  SECTION("Message count tracking")
  {
    // Reset for clean test
    sub_node->reset_for_new_test();
    REQUIRE(sub_node->get_message_count() == 0);

    // Allow time for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Publish multiple messages
    for (int i = 0; i < 3; ++i) {
      std_msgs::msg::String msg;
      msg.data = "Test " + std::to_string(i);
      pub_node->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait for messages to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    REQUIRE(sub_node->get_message_count() == 3);

    // Get last message
    auto last_msg = sub_node->get_last_message();
    REQUIRE(last_msg.has_value());
    REQUIRE(last_msg->data == "Test 2");
  }
}

}  // namespace wato
