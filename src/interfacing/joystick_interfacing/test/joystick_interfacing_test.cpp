
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
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::Joy;
using std_msgs::msg::Bool;
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
     {"max_speed", 2.0},
     {"max_steering_angle", 0.5},
     {"invert_steering", false},
     {"invert_throttle", false}});

  auto joy_node = std::make_shared<joystick_node::JoystickNode>(options);
  add_node(joy_node);

  // Input
  auto joy_pub = std::make_shared<PublisherTestNode<Joy>>("/joy", "joy_pub");
  add_node(joy_pub);

  // Output
  auto ack_sub = std::make_shared<SubscriberTestNode<AckermannDriveStamped>>("/joystick/ackermann", "ack_sub");
  auto idle_sub = std::make_shared<SubscriberTestNode<Bool>>("/joystick/is_idle", "idle_sub");
  add_node(ack_sub);
  add_node(idle_sub);

  start_spinning();

  // Wait for discovery
  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto send_joy = [&](float enable, float steer, float throttle) {
    Joy msg;
    msg.axes.resize(6, 0.0);
    msg.axes[4] = enable;  // enable_axis
    msg.axes[0] = steer;  // steering
    msg.axes[1] = throttle;  // throttle
    joy_pub->publish(msg);
  };

  SECTION("Safety Disengaged (Idle)")
  {
    // Queue expectations BEFORE action to avoid race condition
    auto idle_future = idle_sub->expect_next_message();
    auto ack_future = ack_sub->expect_next_message();

    // Enable not pressed (0.0)
    send_joy(0.0, 1.0, 1.0);

    auto idle_msg = idle_future.get();
    REQUIRE(idle_msg.data == true);

    auto ack_msg = ack_future.get();
    REQUIRE(ack_msg.drive.speed == Catch::Approx(0.0));
  }

  SECTION("Safety Engaged (Active)")
  {
    auto idle_future = idle_sub->expect_next_message();
    auto ack_future = ack_sub->expect_next_message();

    // Enable pressed (-1.0)
    // Steer 1.0 -> max_steering_angle (0.5)
    // Throttle 1.0 -> max_speed (2.0)
    send_joy(-1.0, 1.0, 1.0);

    auto idle_msg = idle_future.get();
    REQUIRE(idle_msg.data == false);  // Not idle

    auto ack_msg = ack_future.get();
    REQUIRE(ack_msg.drive.steering_angle == Catch::Approx(0.5));
    REQUIRE(ack_msg.drive.speed == Catch::Approx(2.0));
  }
}
