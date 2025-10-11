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
#include <memory>
#include <string>
#include <thread>

#include <catch2/catch.hpp>
#include <wato_test/wato_test.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace wato
{

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "Using TestExecutorFixture from wato_test", "[deep_fixture]")
{
  // Create a test node
  auto test_node = std::make_shared<rclcpp::Node>("fixture_test_node");

  // Add it to the executor
  add_node(test_node);

  // Start spinning
  start_spinning();

  SECTION("Node is created and running")
  {
    REQUIRE(test_node != nullptr);
    REQUIRE(test_node->get_name() == std::string("fixture_test_node"));
  }

  SECTION("Can create publishers and subscribers")
  {
    bool message_received = false;
    std::string received_data;

    // Create subscriber
    auto sub = test_node->create_subscription<std_msgs::msg::String>(
      "test_topic", 10, [&message_received, &received_data](const std_msgs::msg::String::SharedPtr msg) {
        message_received = true;
        received_data = msg->data;
      });

    // Create publisher
    auto pub = test_node->create_publisher<std_msgs::msg::String>("test_topic", 10);

    // Wait for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish a message
    std_msgs::msg::String msg;
    msg.data = "Hello from wato_test fixture!";
    pub->publish(msg);

    // Wait for message to be received
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    REQUIRE(message_received == true);
    REQUIRE(received_data == "Hello from wato_test fixture!");
  }

  SECTION("Can create and use services")
  {
    bool service_called = false;

    // Create service
    auto service = test_node->create_service<std_srvs::srv::SetBool>(
      "test_service",
      [&service_called](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        service_called = true;
        response->success = request->data;
        response->message = request->data ? "Service enabled" : "Service disabled";
      });

    // Create client
    auto client = test_node->create_client<std_srvs::srv::SetBool>("test_service");

    // Wait for service to be available
    REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

    // Call the service
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);

    // Wait for response
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto response = future.get();
      REQUIRE(response->success == true);
      REQUIRE(response->message == "Service enabled");
      REQUIRE(service_called == true);
    } else {
      FAIL("Service call timed out");
    }
  }
}

}  // namespace wato
