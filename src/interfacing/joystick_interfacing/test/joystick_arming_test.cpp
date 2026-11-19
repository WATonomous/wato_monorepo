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

#include <catch2/catch_test_macros.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "joystick_interfacing/joystick_interfacing.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using sensor_msgs::msg::Joy;
using sensor_msgs::msg::JoyFeedback;
using std_msgs::msg::Bool;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "Joystick Arming Feature", "[joystick_arming]")
{
  // System Under Test (SUT)
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"enable_axis", 4},
     {"steering_axis", 2},
     {"throttle_axis", 3},
     {"toggle_button", 1},  // Button B
     {"arming_button", 0},  // Button A
     {"ackermann_max_speed", 2.0},
     {"ackermann_max_steering_angle", 0.5},
     {"roscco_max_speed", 2.0},
     {"roscco_max_steering_angle", 0.5},
     {"toggle_vibration_intensity", 1.0},
     {"toggle_vibration_duration_ms", 100}});

  auto joy_node = std::make_shared<joystick_node::JoystickNode>(options);
  add_node(joy_node);

  // Transition lifecycle node to active state
  joy_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  joy_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Input: Joy
  auto joy_pub = std::make_shared<PublisherTestNode<Joy>>("/joy", "joy_pub");
  add_node(joy_pub);

  // Input: is_armed mock. Confirm reliable delivery before processing its callback.
  auto mock_service_node = std::make_shared<rclcpp::Node>("mock_service_node");
  auto is_armed_pub =
    mock_service_node->create_publisher<Bool>("/oscc_interfacing/is_armed", rclcpp::QoS(10).reliable());

  // Output: Feedback
  auto feedback_sub = std::make_shared<SubscriberTestNode<JoyFeedback>>("/joy/set_feedback", "feedback_sub");
  add_node(feedback_sub);

  // Service Mock: /oscc_interfacing/arm
  auto request_promise = std::make_shared<std::promise<bool>>();
  auto request_future = request_promise->get_future();
  auto service_server = mock_service_node->create_service<std_srvs::srv::SetBool>(
    "/oscc_interfacing/arm",
    [request_promise](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      response->success = true;
      response->message = "Mock Success";
      request_promise->set_value(request->data);
    });
  add_node(mock_service_node);

  REQUIRE(joy_pub->wait_for_subscribers());
  REQUIRE(feedback_sub->wait_for_publishers());
  auto service_probe = joy_node->create_client<std_srvs::srv::SetBool>("/oscc_interfacing/arm");
  REQUIRE(service_probe->wait_for_service(std::chrono::seconds(5)));
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (is_armed_pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(is_armed_pub->get_subscription_count() > 0);

  auto send_joy_arm = [&](bool press) {
    Joy msg;
    msg.axes.resize(6, 0.0);
    msg.buttons.resize(2, 0);
    msg.axes[4] = -1.0;  // Enable held
    msg.buttons[0] = press ? 1 : 0;  // Arming button (A)
    msg.buttons[1] = 0;  // Toggle button (B)
    joy_pub->publish(msg);
  };

  SECTION("Arming (OFF -> ON)")
  {
    // 1. Simulate is_armed = false
    {
      Bool msg;
      msg.data = false;
      is_armed_pub->publish(msg);
      // Delivery acknowledgment is not callback completion. Process the queued
      // state before starting the background executor and sending the button.
      REQUIRE(is_armed_pub->wait_for_all_acked(std::chrono::seconds(5)));
      executor_.spin_some();
    }

    start_spinning();

    // 2. Press Arm button
    auto feedback_future = feedback_sub->expect_next_message();  // Expect vibrate start
    send_joy_arm(true);

    // 3. Verify Service Call
    REQUIRE(request_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    REQUIRE(request_future.get() == true);  // Requesting ARM (true)

    // 4. Verify the first vibration pulse. Teardown must safely stop any
    // remaining vibration callbacks; the test does not need to wait for them.
    REQUIRE(feedback_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    auto fb_msg_1 = feedback_future.get();
    REQUIRE(fb_msg_1.type == JoyFeedback::TYPE_RUMBLE);
    REQUIRE(fb_msg_1.intensity > 0.1f);

    // Release button
    send_joy_arm(false);
  }

  SECTION("Disarming (ON -> OFF)")
  {
    // 1. Simulate is_armed = true
    {
      Bool msg;
      msg.data = true;
      is_armed_pub->publish(msg);
      REQUIRE(is_armed_pub->wait_for_all_acked(std::chrono::seconds(5)));
      executor_.spin_some();
    }

    start_spinning();

    // 2. Press Arm button
    send_joy_arm(true);

    // 3. Verify Service Call
    REQUIRE(request_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    REQUIRE(request_future.get() == false);  // Requesting DISARM (false)

    // Release button
    send_joy_arm(false);
  }
}
