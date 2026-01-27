
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

// Copyright (c) 2025-present WATonomous. All rights reserved.

#include <chrono>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
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

  // Input: Joy
  auto joy_pub = std::make_shared<PublisherTestNode<Joy>>("/joy", "joy_pub");
  add_node(joy_pub);

  // Input: is_armed mock
  auto is_armed_pub = std::make_shared<PublisherTestNode<Bool>>("/oscc_interfacing/is_armed", "is_armed_pub");
  add_node(is_armed_pub);

  // Output: Feedback
  auto feedback_sub = std::make_shared<SubscriberTestNode<JoyFeedback>>("/joy/set_feedback", "feedback_sub");
  add_node(feedback_sub);

  // Service Mock: /oscc_interfacing/arm
  auto mock_service_node = std::make_shared<rclcpp::Node>("mock_service_node");
  bool service_called = false;
  bool requested_state = false;
  auto service_server = mock_service_node->create_service<std_srvs::srv::SetBool>(
    "/oscc_interfacing/arm",
    [&](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      service_called = true;
      requested_state = request->data;
      response->success = true;
      response->message = "Mock Success";
    });
  add_node(mock_service_node);

  start_spinning();
  std::this_thread::sleep_for(std::chrono::seconds(1));

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
      // Allow callback to process
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 2. Press Arm button
    auto feedback_future = feedback_sub->expect_next_message();  // Expect vibrate start
    send_joy_arm(true);

    // 3. Verify Service Call
    // Wait a bit for service to handle
    int retries = 0;
    while (!service_called && retries < 10) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      retries++;
    }

    REQUIRE(service_called == true);
    REQUIRE(requested_state == true);  // Requesting ARM (true)

    // 4. Verify Feedback (Double Vibrate)
    // First pulse ON
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
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    service_called = false;

    // 2. Press Arm button
    send_joy_arm(true);

    // 3. Verify Service Call
    int retries = 0;
    while (!service_called && retries < 10) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      retries++;
    }

    REQUIRE(service_called == true);
    REQUIRE(requested_state == false);  // Requesting DISARM (false)

    // Release button
    send_joy_arm(false);
  }
}
