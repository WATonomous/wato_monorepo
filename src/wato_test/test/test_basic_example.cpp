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

#include <catch2/catch_all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <wato_test/wato_test.hpp>

namespace wato
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

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
    // Use promise/future for event-driven message receipt
    std::promise<std::string> message_promise;
    auto message_future = message_promise.get_future();

    // Create subscriber that fulfills promise on first message
    auto sub = test_node->create_subscription<std_msgs::msg::String>(
      "test_topic", 10, [&message_promise](const std_msgs::msg::String::SharedPtr msg) {
        try {
          message_promise.set_value(msg->data);
        } catch (const std::future_error &) {
          // Promise already satisfied, ignore subsequent messages
        }
      });

    // Create publisher
    auto pub = test_node->create_publisher<std_msgs::msg::String>("test_topic", 10);

    // Wait for pub/sub connection (event-driven: check subscriber count)
    auto start = std::chrono::steady_clock::now();
    while (pub->get_subscription_count() == 0) {
      if (std::chrono::steady_clock::now() - start > 1s) {
        FAIL("Timed out waiting for subscriber connection");
      }
      std::this_thread::sleep_for(1ms);
    }

    // Publish a message
    std_msgs::msg::String msg;
    msg.data = "Hello from wato_test fixture!";
    pub->publish(msg);

    // Wait for message with timeout (event-driven via future)
    auto status = message_future.wait_for(1s);
    REQUIRE(status == std::future_status::ready);
    REQUIRE(message_future.get() == "Hello from wato_test fixture!");
  }

  SECTION("Can create and use services")
  {
    std::promise<bool> service_called_promise;
    auto service_called_future = service_called_promise.get_future();

    // Create service
    auto service = test_node->create_service<std_srvs::srv::SetBool>(
      "test_service",
      [&service_called_promise](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        response->success = request->data;
        response->message = request->data ? "Service enabled" : "Service disabled";
        try {
          service_called_promise.set_value(true);
        } catch (const std::future_error &) {
          // Already set
        }
      });

    // Create client
    auto client = test_node->create_client<std_srvs::srv::SetBool>("test_service");

    // Wait for service to be available with shorter timeout
    if (!client->wait_for_service(500ms)) {
      FAIL("Service not available within timeout");
    }

    // Call the service
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);

    // Wait for response with reasonable timeout
    if (future.wait_for(500ms) == std::future_status::ready) {
      auto response = future.get();
      REQUIRE(response->success == true);
      REQUIRE(response->message == "Service enabled");

      // Verify service was actually called
      REQUIRE(service_called_future.wait_for(100ms) == std::future_status::ready);
    } else {
      FAIL("Service call timed out");
    }
  }
}

}  // namespace wato
