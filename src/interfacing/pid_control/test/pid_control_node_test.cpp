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

#include "pid_control/pid_control_node.hpp"

#include <chrono>
#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/roscco.hpp>
#include <std_msgs/msg/float64.hpp>

#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using RosccoMsg = roscco_msg::msg::Roscco;
using std_msgs::msg::Float64;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "PidControlNode Basic Operation", "[pid_control]")
{
  // System Under Test (SUT)
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args",
     "-r",
     "ackermann:=/joystick/ackermann",
     "-r",
     "steering_feedback:=/steering_meas",
     "-r",
     "velocity_feedback:=/velocity_meas",
     "-r",
     "roscco:=/joystick/roscco"});
  options.parameter_overrides(
    {{"update_rate", 10.0},
     {"steering_wheel_conversion_factor", 10.0},
     {"steering_pid.p", 1.0},
     {"steering_pid.i", 0.0},
     {"steering_pid.d", 0.0},
     {"velocity_pid.p", 1.0},
     {"velocity_pid.i", 0.0},
     {"velocity_pid.d", 0.0}});

  auto pid_node = std::make_shared<pid_control::PidControlNode>(options);
  add_node(pid_node);

  // Configure the node (creates PID controllers, subscriptions, publisher)
  // but do NOT activate yet - each SECTION sets parameters first to avoid
  // racing set_parameter() on the test thread with compute_command() on the executor thread.
  pid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Inputs
  auto ackermann_pub =
    std::make_shared<PublisherTestNode<AckermannDriveStamped>>("/joystick/ackermann", "ackermann_pub");
  auto steering_meas_pub = std::make_shared<PublisherTestNode<Float64>>("/steering_meas", "steering_meas_pub");
  auto velocity_meas_pub = std::make_shared<PublisherTestNode<Float64>>("/velocity_meas", "velocity_meas_pub");
  add_node(ackermann_pub);
  add_node(steering_meas_pub);
  add_node(velocity_meas_pub);

  // Output
  auto roscco_sub = std::make_shared<SubscriberTestNode<RosccoMsg>>("/joystick/roscco", "roscco_sub");
  add_node(roscco_sub);

  SECTION("Verify response to error with different P gain")
  {
    // Set parameters before activation (no race with control loop)
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.p", 0.5));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.p", 0.5));

    pid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    // Wait for DDS discovery
    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(steering_meas_pub->wait_for_subscribers(1));
    REQUIRE(velocity_meas_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    // Expect Roscco message
    // Since P=0.5, and error is (0.5 - 0.1) and 1.0, we expect steering torque ~0.2 and forward ~0.5
    auto roscco_future = roscco_sub->expect_next_message();

    // Send Ackermann setpoint (Steering: 0.5 rad, Speed: 1.0 m/s)
    AckermannDriveStamped ack_msg;
    ack_msg.drive.steering_angle = 0.5;
    ack_msg.drive.speed = 1.0;
    ackermann_pub->publish(ack_msg);

    // Send Feedback (Steering: 57.2957795 degrees on wheel, Speed: 0.0)
    // 57.2957795 / 10.0 = 5.72957795 degrees = 0.1 radians
    // Error = 0.5 - 0.1 = 0.4
    Float64 steer_msg;
    steer_msg.data = 57.2957795;
    steering_meas_pub->publish(steer_msg);

    Float64 vel_msg;
    vel_msg.data = 0.0;
    velocity_meas_pub->publish(vel_msg);

    auto result_msg = roscco_future.get();

    REQUIRE(result_msg.steering == Catch::Approx(0.2));
    REQUIRE(result_msg.forward == Catch::Approx(0.5));
  }

  SECTION("Verify Integral (I) term accumulation")
  {
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.p", 0.0));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.i", 1.0));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.p", 0.0));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.i", 1.0));

    pid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(steering_meas_pub->wait_for_subscribers(1));
    REQUIRE(velocity_meas_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    // Get two consecutive messages and check if the second one has a larger command
    auto future1 = roscco_sub->expect_next_message();

    // Send setpoints and constant measurements
    AckermannDriveStamped ack_msg;
    ack_msg.drive.steering_angle = 1.0;
    ack_msg.drive.speed = 1.0;
    ackermann_pub->publish(ack_msg);

    Float64 meas_msg;
    meas_msg.data = 0.0;
    steering_meas_pub->publish(meas_msg);
    velocity_meas_pub->publish(meas_msg);

    auto msg1 = future1.get();

    // Wait a bit to let the integrator accumulate
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto future2 = roscco_sub->expect_next_message();
    auto msg2 = future2.get();

    // With I > 0 and error > 0, the command should be strictly increasing
    REQUIRE(msg2.steering > msg1.steering);
    REQUIRE(msg2.forward > msg1.forward);
  }

  SECTION("Verify Derivative (D) term response")
  {
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.p", 0.0));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.d", 1.0));

    pid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(steering_meas_pub->wait_for_subscribers(1));
    REQUIRE(velocity_meas_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    // Set point at 0
    AckermannDriveStamped ack_msg;
    ack_msg.drive.steering_angle = 0.0;
    ack_msg.drive.speed = 0.0;
    ackermann_pub->publish(ack_msg);

    // Initial feedback at 0 (No error, no change)
    Float64 meas_msg;
    meas_msg.data = 0.0;
    steering_meas_pub->publish(meas_msg);
    velocity_meas_pub->publish(meas_msg);

    auto future1 = roscco_sub->expect_next_message();
    future1.get();  // Flush initial zero command

    auto future2 = roscco_sub->expect_next_message();

    // Suddenly change measurement to 1.0 (Error change: 0 -> -1.0)
    // Derivative term D * (dError/dt) should be negative
    meas_msg.data = 1.0;
    steering_meas_pub->publish(meas_msg);

    auto msg2 = future2.get();

    REQUIRE(msg2.steering < 0.0);
  }

  SECTION("Verify I-clamp (Anti-Windup) behavior")
  {
    // Steering: 0.5 clamp
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.p", 0.0));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.i", 10.0));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.u_clamp_max", 0.5));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.u_clamp_min", -0.5));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.saturation", true));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.antiwindup", true));
    pid_node->set_parameter(rclcpp::Parameter("steering_pid.antiwindup_strategy", "conditional_integration"));

    // Velocity: 0.8 clamp
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.p", 0.0));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.i", 10.0));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.u_clamp_max", 0.8));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.u_clamp_min", -0.8));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.saturation", true));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.antiwindup", true));
    pid_node->set_parameter(rclcpp::Parameter("velocity_pid.antiwindup_strategy", "conditional_integration"));

    pid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    start_spinning();

    REQUIRE(ackermann_pub->wait_for_subscribers(1));
    REQUIRE(steering_meas_pub->wait_for_subscribers(1));
    REQUIRE(velocity_meas_pub->wait_for_subscribers(1));
    REQUIRE(roscco_sub->wait_for_publishers(1));

    auto future = roscco_sub->expect_next_message();

    // Send constant error
    AckermannDriveStamped ack_msg;
    ack_msg.drive.steering_angle = 1.0;
    ack_msg.drive.speed = 1.0;
    ackermann_pub->publish(ack_msg);

    Float64 meas_msg;
    meas_msg.data = 0.0;
    steering_meas_pub->publish(meas_msg);
    velocity_meas_pub->publish(meas_msg);

    // Integrate for 0.2s. 10.0/s * 0.2s = 2.0 (unclamped)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto result_msg = future.get();

    // Verify clamping
    REQUIRE(result_msg.steering <= 0.55);
    REQUIRE(result_msg.forward <= 0.85);
  }
}
