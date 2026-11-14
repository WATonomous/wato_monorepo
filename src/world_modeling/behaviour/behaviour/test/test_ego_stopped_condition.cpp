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

#include <rcl/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>

#include <catch2/catch_all.hpp>

#include "behaviour/nodes/common/conditions/ego_stopped_condition.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

namespace
{
struct EgoStoppedFixture : wato::test::ROS2Initializer
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_ego_stopped");
  BT::Blackboard::Ptr root = BT::Blackboard::create();
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create(root);
  std::unique_ptr<behaviour::EgoStoppedCondition> condition;
  nav_msgs::msg::Odometry::SharedPtr odom;

  EgoStoppedFixture()
  {
    root->set("node", node);
    blackboard->set("threshold", 0.1);
    blackboard->set("window", 3);
    BT::NodeConfig config;
    config.blackboard = blackboard;
    config.input_ports = {{"ego_odom", "{odom}"}, {"threshold_velocity", "{threshold}"}, {"window_size", "{window}"}};
    condition = std::make_unique<behaviour::EgoStoppedCondition>("EgoStopped", config, node->get_logger());
  }

  BT::NodeStatus sample(double vx = 0.0, double vy = 0.0)
  {
    odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = node->now();
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    blackboard->set("odom", odom);
    return condition->tick();
  }

  void warm()
  {
    REQUIRE(sample() == BT::NodeStatus::FAILURE);
    REQUIRE(sample() == BT::NodeStatus::FAILURE);
    REQUIRE(sample() == BT::NodeStatus::SUCCESS);
  }
};
}  // namespace

TEST_CASE_METHOD(EgoStoppedFixture, "Repeated ticks do not become independent velocity samples", "[ego_stopped]")
{
  REQUIRE(sample() == BT::NodeStatus::FAILURE);
  for (int i = 0; i < 100; ++i) {
    REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
  }
  REQUIRE(sample() == BT::NodeStatus::FAILURE);
  REQUIRE(sample() == BT::NodeStatus::SUCCESS);
  REQUIRE(sample(1000.0) == BT::NodeStatus::SUCCESS);
  for (int i = 0; i < 100; ++i) {
    REQUIRE(condition->tick() == BT::NodeStatus::SUCCESS);
  }
  REQUIRE(sample(1000.0) == BT::NodeStatus::FAILURE);
}

TEST_CASE_METHOD(EgoStoppedFixture, "Three samples reject one spike but recognize sustained movement", "[ego_stopped]")
{
  warm();
  REQUIRE(sample(1e100) == BT::NodeStatus::SUCCESS);
  REQUIRE(sample() == BT::NodeStatus::SUCCESS);
  REQUIRE(sample() == BT::NodeStatus::SUCCESS);
  const double vx = GENERATE(-2.0, 0.0, 0.08);
  const double vy = vx == 0.0 ? -2.0 : (vx == 0.08 ? 0.08 : 0.0);
  REQUIRE(sample(vx, vy) == BT::NodeStatus::SUCCESS);
  REQUIRE(sample(vx, vy) == BT::NodeStatus::FAILURE);
  REQUIRE(sample() == BT::NodeStatus::FAILURE);
  REQUIRE(sample() == BT::NodeStatus::SUCCESS);
}

TEST_CASE_METHOD(EgoStoppedFixture, "The existing window size remains configurable", "[ego_stopped]")
{
  const int window = GENERATE(0, 1, 2, 5);
  blackboard->set("window", window);
  const int count = std::max(1, window);
  for (int i = 1; i < count; ++i) {
    REQUIRE(sample() == BT::NodeStatus::FAILURE);
  }
  REQUIRE(sample() == BT::NodeStatus::SUCCESS);
  // Upper median for even windows also requires a strict majority of low samples.
  for (int i = 1; i < (count + 1) / 2; ++i) {
    REQUIRE(sample(100.0) == BT::NodeStatus::SUCCESS);
  }
  REQUIRE(sample(100.0) == BT::NodeStatus::FAILURE);
}

TEST_CASE_METHOD(EgoStoppedFixture, "Missing and invalid inputs discard the previous history", "[ego_stopped]")
{
  warm();
  SECTION("Missing odometry")
  {
    blackboard->set<nav_msgs::msg::Odometry::SharedPtr>("odom", nullptr);
  }
  SECTION("Missing threshold")
  {
    blackboard->unset("threshold");
  }
  SECTION("Invalid threshold")
  {
    blackboard->set("threshold", GENERATE(-0.1, std::numeric_limits<double>::quiet_NaN()));
  }
  SECTION("Non-finite velocity")
  {
    odom->twist.twist.linear.y =
      GENERATE(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity());
  }
  REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
  blackboard->set("threshold", 0.1);
  warm();
}

TEST_CASE_METHOD(EgoStoppedFixture, "Stale, future and malformed stamps fail closed", "[ego_stopped]")
{
  warm();
  SECTION("Stale")
  {
    odom->header.stamp = node->now() - rclcpp::Duration::from_seconds(0.6);
  }
  SECTION("Future")
  {
    odom->header.stamp = node->now() + rclcpp::Duration::from_seconds(1.0);
  }
  SECTION("Invalid nanoseconds")
  {
    odom->header.stamp.nanosec = 1'000'000'000U;
  }
  SECTION("Zero stamp")
  {
    odom->header.stamp.sec = 0;
    odom->header.stamp.nanosec = 0;
  }
  REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
  warm();
}

TEST_CASE_METHOD(EgoStoppedFixture, "Backwards timestamps discard history and recover", "[ego_stopped]")
{
  warm();
  odom->header.stamp = rclcpp::Time(odom->header.stamp) - rclcpp::Duration::from_nanoseconds(1);
  REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
  warm();
}

TEST_CASE_METHOD(EgoStoppedFixture, "Frozen messages expire even while ROS time is paused", "[ego_stopped]")
{
  auto * clock = node->get_clock()->get_clock_handle();
  REQUIRE(rcl_enable_ros_time_override(clock) == RCL_RET_OK);
  for (int64_t i = 0; i < 3; ++i) {
    REQUIRE(rcl_set_ros_time_override(clock, 10'000'000'000LL + i) == RCL_RET_OK);
    REQUIRE(sample() == (i < 2 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
  blackboard->set("window", 1);
  REQUIRE(condition->tick() == BT::NodeStatus::FAILURE);
}
