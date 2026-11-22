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
#include <cmath>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <string>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "fake_planner/fake_planner_node.hpp"

namespace wato
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

static rclcpp::NodeOptions plannerOptions()
{
  const auto maneuver =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "maneuvers/steady_state_slalom.json";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("maneuver_file", maneuver.string()),
    rclcpp::Parameter("start_on_activate", false),
    rclcpp::Parameter("release_ramp_s", 0.0),
  });
  options.arguments({
    "--ros-args",
    "-r",
    "__ns:=/fake_planner_test",
    "-r",
    "trajectory:=/test/trajectory",
    "-r",
    "odom:=/test/odom",
    "-r",
    "trajectory_markers:=/test/markers",
  });
  return options;
}

TEST_CASE_METHOD(
  test::TestExecutorFixture, "Lifecycle configuration validates parameters and can be repeated", "[fake_planner_node]")
{
  auto node = std::make_shared<fake_planner::FakePlannerNode>(plannerOptions());
  add_node(node);
  CHECK_FALSE(node->has_parameter("trajectory_topic"));
  CHECK_FALSE(node->has_parameter("odom_topic"));
  CHECK_FALSE(node->has_parameter("behaviour_topic"));
  CHECK_FALSE(node->has_parameter("marker_pub_topic"));

  node->set_parameter(rclcpp::Parameter("publish_rate_hz", std::numeric_limits<double>::infinity()));
  REQUIRE(node->configure().label() == "unconfigured");
  node->set_parameter(rclcpp::Parameter("publish_rate_hz", 10.0));
  REQUIRE(node->configure().label() == "inactive");
  REQUIRE(node->activate().label() == "active");
  REQUIRE(node->deactivate().label() == "inactive");
  REQUIRE(node->cleanup().label() == "unconfigured");
  REQUIRE(node->configure().label() == "inactive");
  REQUIRE(node->cleanup().label() == "unconfigured");
}

TEST_CASE_METHOD(
  test::TestExecutorFixture,
  "Remapped trajectory waits for release and resets its odometry anchor",
  "[fake_planner_node]")
{
  using Trajectory = wato_trajectory_msgs::msg::Trajectory;
  using Markers = visualization_msgs::msg::MarkerArray;
  auto node = std::make_shared<fake_planner::FakePlannerNode>(plannerOptions());
  auto odometry = std::make_shared<test::PublisherTestNode<nav_msgs::msg::Odometry>>("/test/odom", "odom_source");
  auto trajectory = std::make_shared<test::SubscriberTestNode<Trajectory>>("/test/trajectory", "trajectory_receiver");
  auto markers = std::make_shared<test::SubscriberTestNode<Markers>>("/test/markers", "marker_receiver");
  auto start = std::make_shared<test::ClientTestNode<std_srvs::srv::Trigger>>(
    "/fake_planner_test/fake_planner_node/start_trajectory", "start_client");
  auto stop = std::make_shared<test::ClientTestNode<std_srvs::srv::Trigger>>(
    "/fake_planner_test/fake_planner_node/stop_trajectory", "stop_client");
  auto reset = std::make_shared<test::ClientTestNode<std_srvs::srv::Trigger>>(
    "/fake_planner_test/fake_planner_node/reset", "reset_client");
  add_node(node);
  add_node(odometry);
  add_node(trajectory);
  add_node(markers);
  add_node(start);
  add_node(stop);
  add_node(reset);
  REQUIRE(node->configure().label() == "inactive");
  REQUIRE(node->activate().label() == "active");
  start_spinning();
  REQUIRE(odometry->wait_for_subscribers(1));
  REQUIRE(trajectory->wait_for_publishers(1));
  REQUIRE(markers->wait_for_publishers(1));

  auto preview = markers->expect_message([](const Markers & message) {
    for (const auto & marker : message.markers) {
      if (marker.ns == "trajectory_velocity" && marker.id == 0) {
        return std::abs(marker.pose.position.x - 10.0) < 0.01;
      }
    }
    return false;
  });
  auto first = trajectory->expect_next_message();
  nav_msgs::msg::Odometry pose;
  pose.pose.pose.position.x = 10.0;
  pose.pose.pose.position.y = 20.0;
  pose.pose.pose.orientation.w = 1.0;
  odometry->publish(pose);
  REQUIRE(preview.wait_for(5s) == std::future_status::ready);
  REQUIRE(first.wait_for(150ms) == std::future_status::timeout);

  auto started = start->send_request_async(std_srvs::srv::Trigger::Request{});
  REQUIRE(started.wait_for(5s) == std::future_status::ready);
  REQUIRE(started.get().success);
  REQUIRE(first.wait_for(5s) == std::future_status::ready);
  const auto published = first.get();
  REQUIRE_FALSE(published.points.empty());
  CHECK(published.points.front().pose.position.x == Catch::Approx(10.0));
  CHECK(published.points.front().pose.position.y == Catch::Approx(20.0));

  auto reset_result = reset->send_request_async(std_srvs::srv::Trigger::Request{});
  REQUIRE(reset_result.wait_for(5s) == std::future_status::ready);
  REQUIRE(reset_result.get().success);
  auto second = trajectory->expect_message([](const Trajectory & message) {
    return !message.points.empty() && message.points.front().pose.position.x > 99.0;
  });
  preview = markers->expect_message([](const Markers & message) {
    for (const auto & marker : message.markers) {
      if (marker.ns == "trajectory_velocity" && marker.id == 0) {
        return std::abs(marker.pose.position.x - 100.0) < 0.01;
      }
    }
    return false;
  });
  pose.pose.pose.position.x = 100.0;
  pose.pose.pose.position.y = 200.0;
  odometry->publish(pose);
  REQUIRE(preview.wait_for(5s) == std::future_status::ready);
  REQUIRE(second.wait_for(150ms) == std::future_status::timeout);
  started = start->send_request_async(std_srvs::srv::Trigger::Request{});
  REQUIRE(started.wait_for(5s) == std::future_status::ready);
  REQUIRE(started.get().success);
  REQUIRE(second.wait_for(5s) == std::future_status::ready);
  const auto replayed = second.get();
  CHECK(replayed.points.front().pose.position.x == Catch::Approx(100.0));
  CHECK(replayed.points.front().pose.position.y == Catch::Approx(200.0));

  auto stopped = stop->send_request_async(std_srvs::srv::Trigger::Request{});
  REQUIRE(stopped.wait_for(5s) == std::future_status::ready);
  REQUIRE(stopped.get().success);
}

}  // namespace wato
