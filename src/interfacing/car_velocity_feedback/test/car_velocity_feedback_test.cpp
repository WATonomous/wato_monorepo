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
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/steering_angle.hpp>
#include <roscco_msg/msg/wheel_speeds.hpp>
#include <std_msgs/msg/float64.hpp>

#include "car_velocity_feedback/car_velocity_feedback_node.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using car_velocity_feedback::CarVelocityFeedbackNode;
using roscco_msg::msg::SteeringAngle;
using roscco_msg::msg::WheelSpeeds;
using std_msgs::msg::Float64;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

TEST_CASE_METHOD(TestExecutorFixture, "Car Velocity Feedback Calculation", "[car_velocity_feedback]")
{
  // System Under Test (SUT)
  rclcpp::NodeOptions options;
  auto feedback_node = std::make_shared<CarVelocityFeedbackNode>(options);
  add_node(feedback_node);

  // Transition lifecycle node to active state
  feedback_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  feedback_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Input Publishers
  auto wheel_speeds_pub =
    std::make_shared<PublisherTestNode<WheelSpeeds>>("/oscc_interfacing/wheel_speeds", "wheel_speeds_pub");
  auto steering_angle_pub =
    std::make_shared<PublisherTestNode<SteeringAngle>>("/oscc_interfacing/steering_angle", "steering_angle_pub");

  add_node(wheel_speeds_pub);
  add_node(steering_angle_pub);

  // Output Subscriber
  auto velocity_sub = std::make_shared<SubscriberTestNode<Float64>>("/body_velocity_feedback", "velocity_sub");
  add_node(velocity_sub);

  start_spinning();

  // Wait for discovery
  REQUIRE(wheel_speeds_pub->wait_for_subscribers(1));
  REQUIRE(steering_angle_pub->wait_for_subscribers(1));
  REQUIRE(velocity_sub->wait_for_publishers(1));

  SECTION("Calculate straight line velocity")
  {
    // Publish steering angle (0 degrees)
    SteeringAngle steering_msg;
    steering_msg.angle = 0.0;
    steering_angle_pub->publish(steering_msg);

    // Queue expectation
    auto velocity_future = velocity_sub->expect_next_message();

    // Publish wheel speeds (36 km/h = 10 m/s)
    WheelSpeeds wheel_msg;
    wheel_msg.ne = 36.0;
    wheel_msg.nw = 36.0;
    wheel_msg.se = 36.0;
    wheel_msg.sw = 36.0;
    wheel_speeds_pub->publish(wheel_msg);

    // Verify
    auto velocity_msg = velocity_future.get();
    // Expect 10.0 m/s
    REQUIRE(velocity_msg.data == Catch::Approx(10.0));
  }

  SECTION("Calculate turning velocity")
  {
    // Publish steering angle (60 degrees = ~1.0472 rad)
    // cos(60) = 0.5
    SteeringAngle steering_msg;
    steering_msg.angle = 1.04719755;
    steering_angle_pub->publish(steering_msg);

    // Queue expectation
    auto velocity_future = velocity_sub->expect_next_message();

    // Publish wheel speeds (36 km/h = 10 m/s)
    WheelSpeeds wheel_msg;
    wheel_msg.ne = 36.0;
    wheel_msg.nw = 36.0;
    wheel_msg.se = 36.0;
    wheel_msg.sw = 36.0;
    wheel_speeds_pub->publish(wheel_msg);

    // Verify
    auto velocity_msg = velocity_future.get();
    // Expect 5.0 m/s (10 * 0.5)
    REQUIRE(velocity_msg.data == Catch::Approx(5.0).epsilon(0.01));
  }
}
