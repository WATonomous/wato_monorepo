
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

#include "joystick_interfacing/joystick_interfacing.hpp"

#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using RosccoMsg = roscco_msg::msg::Roscco;
using sensor_msgs::msg::Joy;
using sensor_msgs::msg::JoyFeedback;
using std_msgs::msg::Bool;
using std_msgs::msg::Int8;
using JoystickState = joystick_node::JoystickNode::JoystickState;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "Joystick Interfacing Operation", "[joystick_interfacing]")
{
  // System Under Test (SUT)
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"enable_axis", 4},
     {"steering_axis", 0},
     {"throttle_axis", 1},
     {"toggle_button", 0},
     {"arming_button", 5},
     {"ackermann_max_speed", 2.0},
     {"ackermann_max_steering_angle", 0.5},
     {"roscco_max_speed", 2.0},
     {"roscco_max_steering_angle", 0.5},
     {"invert_steering", false},
     {"invert_throttle", false}});

  auto joy_node = std::make_shared<joystick_node::JoystickNode>(options);
  add_node(joy_node);

  // Input
  auto joy_pub = std::make_shared<PublisherTestNode<Joy>>("/joy", "joy_pub");
  add_node(joy_pub);

  // Output
  auto ack_sub = std::make_shared<SubscriberTestNode<AckermannDriveStamped>>("/joystick/ackermann", "ack_sub");
  auto roscco_sub = std::make_shared<SubscriberTestNode<RosccoMsg>>("/joystick/roscco", "roscco_sub");
  auto idle_sub = std::make_shared<SubscriberTestNode<Bool>>("/joystick/is_idle", "idle_sub");
  auto state_sub = std::make_shared<SubscriberTestNode<Int8>>("/joystick/state", "state_sub");
  auto feedback_sub = std::make_shared<SubscriberTestNode<JoyFeedback>>("/joy/set_feedback", "feedback_sub");
  add_node(ack_sub);
  add_node(roscco_sub);
  add_node(idle_sub);
  add_node(state_sub);
  add_node(feedback_sub);

  start_spinning();

  // Wait for discovery
  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto send_joy = [&](float enable, float steer, float throttle, bool toggle = false) {
    Joy msg;
    msg.buttons.resize(6, 0);
    msg.axes.resize(6, 0.0);
    msg.axes[4] = enable;  // enable_axis
    msg.axes[0] = steer;  // steering
    msg.axes[1] = throttle;  // throttle
    msg.buttons[0] = toggle ? 1 : 0;  // toggle_button
    joy_pub->publish(msg);
  };

  SECTION("Safety Disengaged (Idle)")
  {
    // Queue expectations BEFORE action to avoid race condition
    auto idle_future = idle_sub->expect_next_message();
    auto state_future = state_sub->expect_next_message();
    auto ack_future = ack_sub->expect_next_message();

    // Enable not pressed (0.0)
    send_joy(0.0, 1.0, 1.0);

    auto idle_msg = idle_future.get();
    REQUIRE(idle_msg.data == true);

    auto state_msg = state_future.get();
    REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::NULL_STATE));

    auto ack_msg = ack_future.get();
    REQUIRE(ack_msg.drive.speed == Catch::Approx(0.0));
  }

  SECTION("Safety Engaged (Active)")
  {
    auto idle_future = idle_sub->expect_next_message();
    auto state_future = state_sub->expect_next_message();
    auto ack_future = ack_sub->expect_next_message();

    // Enable pressed (-1.0)
    // Steer 1.0 -> max_steering_angle (0.5)
    // Throttle 1.0 -> max_speed (2.0)
    send_joy(-1.0, 1.0, 1.0);

    auto idle_msg = idle_future.get();
    REQUIRE(idle_msg.data == false);  // Not idle

    auto state_msg = state_future.get();
    REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ACKERMANN));

    auto ack_msg = ack_future.get();
    REQUIRE(ack_msg.drive.steering_angle == Catch::Approx(0.5));
    REQUIRE(ack_msg.drive.speed == Catch::Approx(2.0));
  }

  SECTION("Topic Toggle")
  {
    // Initially should be on /joystick/ackermann
    {
      auto state_future = state_sub->expect_next_message();
      auto ack_future = ack_sub->expect_next_message();
      send_joy(-1.0, 0.5, 0.5, false);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ACKERMANN));

      auto ack_msg = ack_future.get();
      REQUIRE(ack_msg.drive.speed == Catch::Approx(1.0));
    }

    // Toggle to /joystick/roscco
    {
      auto state_future = state_sub->expect_next_message();
      auto roscco_future = roscco_sub->expect_next_message();
      auto feedback_on_future = feedback_sub->expect_next_message();
      send_joy(-1.0, 0.5, 0.5, true);  // Rising edge toggles to /joystick/roscco

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ROSSCO));

      auto roscco_msg = roscco_future.get();
      REQUIRE(roscco_msg.forward == Catch::Approx(1.0));

      auto fb_on_msg = feedback_on_future.get();
      REQUIRE(fb_on_msg.intensity == Catch::Approx(0.5));

      // Wait for the rest of the 2nd vibration pulse sequence (OFF -> ON -> OFF)
      auto feedback_off_future = feedback_sub->expect_next_message();
      auto fb_off_msg = feedback_off_future.get();
      REQUIRE(fb_off_msg.intensity == Catch::Approx(0.0));

      auto feedback_on2_future = feedback_sub->expect_next_message();
      auto fb_on2_msg = feedback_on2_future.get();
      REQUIRE(fb_on2_msg.intensity == Catch::Approx(0.5));

      auto feedback_off2_future = feedback_sub->expect_next_message();
      auto fb_off2_msg = feedback_off2_future.get();
      REQUIRE(fb_off2_msg.intensity == Catch::Approx(0.0));
    }

    // Release toggle button
    {
      auto state_future = state_sub->expect_next_message();
      auto roscco_future = roscco_sub->expect_next_message();
      send_joy(-1.0, 0.5, 0.5, false);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ROSSCO));

      auto roscco_msg = roscco_future.get();
      REQUIRE(roscco_msg.forward == Catch::Approx(1.0));
    }

    // Verify it publishes to /joystick/roscco with different values
    {
      auto state_future = state_sub->expect_next_message();
      auto roscco_future = roscco_sub->expect_next_message();
      send_joy(-1.0, 0.8, 0.8, false);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ROSSCO));

      auto roscco_msg = roscco_future.get();
      REQUIRE(roscco_msg.forward == Catch::Approx(1.6));  // 0.8 * 2.0
    }

    // Toggle back to /joystick/ackermann
    {
      auto state_future = state_sub->expect_next_message();
      auto ack_future = ack_sub->expect_next_message();
      auto feedback_on_future = feedback_sub->expect_next_message();
      send_joy(-1.0, 0.0, 0.0, true);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ACKERMANN));

      auto ack_msg = ack_future.get();
      REQUIRE(ack_msg.drive.speed == Catch::Approx(0.0));

      auto fb_on_msg = feedback_on_future.get();
      REQUIRE(fb_on_msg.intensity == Catch::Approx(0.5));

      auto feedback_off_future = feedback_sub->expect_next_message();
      auto fb_off_msg = feedback_off_future.get();
      REQUIRE(fb_off_msg.intensity == Catch::Approx(0.0));
    }

    // Release toggle button
    {
      auto state_future = state_sub->expect_next_message();
      auto ack_future = ack_sub->expect_next_message();
      send_joy(-1.0, 0.0, 0.0, false);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ACKERMANN));

      auto ack_msg = ack_future.get();
      REQUIRE(ack_msg.drive.speed == Catch::Approx(0.0));
    }

    // Verify it's back on /joystick/ackermann
    {
      auto state_future = state_sub->expect_next_message();
      auto ack_future = ack_sub->expect_next_message();
      send_joy(-1.0, 0.1, 0.1, false);

      auto state_msg = state_future.get();
      REQUIRE(state_msg.data == static_cast<int8_t>(JoystickState::ACKERMANN));

      auto ack_msg = ack_future.get();
      REQUIRE(ack_msg.drive.speed == Catch::Approx(0.2));  // 0.1 * 2.0
    }
  }
}
