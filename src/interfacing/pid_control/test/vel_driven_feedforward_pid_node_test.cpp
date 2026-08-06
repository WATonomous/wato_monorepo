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

#include "pid_control/vel_driven_feedforward_pid_node.hpp"

#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <catch2/catch_test_macros.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <std_msgs/msg/float64.hpp>

#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using RosccoMsg = roscco_msg::msg::Roscco;
using std_msgs::msg::Float64;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

// Regression coverage for the CAN-vs-GPS velocity feedback race: the old
// "whichever topic publishes first locks the source forever" logic meant an
// ODOM/GPS message arriving before the first CAN message would lock the node
// to GPS permanently. CAN must always be authoritative once it has published.
TEST_CASE_METHOD(TestExecutorFixture, "VelDrivenFeedforwardPidNode Velocity Source Selection", "[pid_control]")
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"update_rate", 10.0}, {"velocity.pid.p", 1.0}, {"velocity.pid.i", 0.0}, {"velocity.pid.d", 0.0}});

  auto node = std::make_shared<pid_control::VelDrivenFeedforwardPidNode>(options);
  add_node(node);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  auto ackermann_pub = std::make_shared<PublisherTestNode<AckermannDriveStamped>>("ackermann", "ackermann_pub");
  auto odom_pub = std::make_shared<PublisherTestNode<Odometry>>("odom_feedback", "odom_pub");
  auto can_pub = std::make_shared<PublisherTestNode<Float64>>("velocity_feedback", "can_pub");
  auto roscco_sub = std::make_shared<SubscriberTestNode<RosccoMsg>>("roscco", "roscco_sub");
  add_node(ackermann_pub);
  add_node(odom_pub);
  add_node(can_pub);
  add_node(roscco_sub);

  SECTION("CAN reclaims the source even after ODOM published first")
  {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(odom_pub->wait_for_subscribers(1));
    REQUIRE(can_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    // Constant setpoint: speed = 1.0 m/s. control_loop() publishes nothing
    // until the first ackermann message arrives, so waiting for the first
    // roscco message here guarantees the setpoint has been processed.
    auto flush_future = roscco_sub->expect_next_message();
    AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = 1.0;
    ackermann_pub->publish(ack_msg);
    flush_future.get();

    // ODOM publishes first and would win under the old "first publisher locks forever" logic.
    auto future1 = roscco_sub->expect_next_message();
    Odometry odom_msg;
    odom_msg.twist.twist.linear.x = 2.0;  // error = 1.0 - 2.0 = -1.0 -> braking (negative forward)
    odom_pub->publish(odom_msg);
    auto msg1 = future1.get();
    REQUIRE(msg1.forward < 0.0);

    // CAN publishes afterward and must reclaim the source, not be locked out by ODOM.
    auto future2 = roscco_sub->expect_next_message();
    Float64 can_msg;
    can_msg.data = 0.0;  // error = 1.0 - 0.0 = 1.0 -> throttle (positive forward)
    can_pub->publish(can_msg);
    auto msg2 = future2.get();
    REQUIRE(msg2.forward > 0.0);
  }

  SECTION("ODOM is used as a bootstrap fallback when CAN never publishes")
  {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(odom_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    auto flush_future = roscco_sub->expect_next_message();
    AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = 1.0;
    ackermann_pub->publish(ack_msg);
    flush_future.get();

    auto future = roscco_sub->expect_next_message();
    Odometry odom_msg;
    odom_msg.twist.twist.linear.x = 0.0;  // error = 1.0 - 0.0 = 1.0 -> throttle (positive forward)
    odom_pub->publish(odom_msg);
    auto msg = future.get();
    REQUIRE(msg.forward > 0.0);
  }
}
