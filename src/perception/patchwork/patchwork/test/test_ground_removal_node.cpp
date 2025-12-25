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

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wato_test/wato_test.hpp>

#include "patchworkpp/ground_removal_node.hpp"

using wato::perception::patchworkpp::GroundRemovalNode;

namespace
{

struct RclcppGuard
{
  RclcppGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

sensor_msgs::msg::PointCloud2 makePointCloud(std::size_t point_count)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(point_count);
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = 3.0f + static_cast<float>(i) * 0.1f;
    *iter_y = static_cast<float>(i % 4);
    *iter_z = 0.0F;
  }

  cloud.header.frame_id = "test_frame";
  cloud.header.stamp = rclcpp::Clock().now();
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeInvalidCloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 5;
  cloud.header.frame_id = "test_frame";
  cloud.header.stamp = rclcpp::Clock().now();
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeCloudWithNonFinite()
{
  auto cloud = makePointCloud(6);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (std::size_t i = 0; i < 6; ++i, ++iter_x, ++iter_y, ++iter_z) {
    if (i == 1) {
      *iter_x = std::numeric_limits<float>::quiet_NaN();
    } else if (i == 3) {
      *iter_y = std::numeric_limits<float>::infinity();
    } else if (i == 5) {
      *iter_z = -std::numeric_limits<float>::infinity();
    }
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makePointCloudFromTriples(const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  cloud.header.frame_id = "test_frame";
  cloud.header.stamp = rclcpp::Clock().now();

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & p : points) {
    *iter_x = p[0];
    *iter_y = p[1];
    *iter_z = p[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

bool hasOnlyFinitePoints(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  const std::size_t total = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  for (std::size_t i = 0; i < total; ++i, ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      return false;
    }
  }
  return true;
}

template <typename Predicate>
bool waitForCondition(Predicate && condition, std::chrono::milliseconds timeout, std::chrono::milliseconds interval)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return condition();
}

template <typename MessageType>
std::optional<MessageType> waitForFuture(std::future<MessageType> future, std::chrono::milliseconds timeout)
{
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  return std::nullopt;
}

}  // namespace

TEST_CASE("Lifecycle transitions succeed without executor", "[node][fast]")
{
  RclcppGuard guard;
  auto node = std::make_shared<GroundRemovalNode>(rclcpp::NodeOptions{});

  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  REQUIRE(node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture, "Handles empty or malformed input safely", "[node][integration][fast]")
{
  auto node = std::make_shared<GroundRemovalNode>(rclcpp::NodeOptions{});
  add_node(node);

  auto input_pub =
    std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kCloudTopic);
  add_node(input_pub);

  node->configure();
  node->activate();
  start_spinning();

  REQUIRE(waitForCondition(
    [&]() { return input_pub->count_subscribers(GroundRemovalNode::kCloudTopic) > 0; },
    std::chrono::milliseconds(500),
    std::chrono::milliseconds(25)));

  SECTION("Empty cloud is ignored without crashing")
  {
    input_pub->publish(makePointCloud(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  SECTION("Cloud missing required fields is handled gracefully")
  {
    input_pub->publish(makeInvalidCloud());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  node->deactivate();
  node->cleanup();
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture, "Publishes segmented clouds and filters invalid input", "[node][integration][fast]")
{
  auto node = std::make_shared<GroundRemovalNode>(rclcpp::NodeOptions{});
  add_node(node);

  auto ground_sub =
    std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kGroundTopic);
  auto nonground_sub =
    std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kNonGroundTopic);
  add_node(ground_sub);
  add_node(nonground_sub);

  auto input_pub =
    std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kCloudTopic);
  add_node(input_pub);

  node->configure();
  node->activate();
  start_spinning();

  REQUIRE(waitForCondition(
    [&]() {
      return input_pub->count_subscribers(GroundRemovalNode::kCloudTopic) > 0 &&
             ground_sub->count_publishers(GroundRemovalNode::kGroundTopic) > 0 &&
             nonground_sub->count_publishers(GroundRemovalNode::kNonGroundTopic) > 0;
    },
    std::chrono::milliseconds(750),
    std::chrono::milliseconds(25)));

  SECTION("Publishes segmented clouds when active")
  {
    auto ground_future = ground_sub->expect_next_message();
    auto nonground_future = nonground_sub->expect_next_message();

    input_pub->publish(makePointCloud(30));

    auto ground_msg = waitForFuture(std::move(ground_future), std::chrono::milliseconds(750));
    auto nonground_msg = waitForFuture(std::move(nonground_future), std::chrono::milliseconds(750));

    REQUIRE(ground_msg.has_value());
    REQUIRE(nonground_msg.has_value());
    REQUIRE(ground_msg->header.frame_id == "test_frame");
    REQUIRE(nonground_msg->header.frame_id == "test_frame");
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  SECTION("Filters non-finite inputs before publishing")
  {
    auto ground_future = ground_sub->expect_next_message();
    auto nonground_future = nonground_sub->expect_next_message();

    input_pub->publish(makeCloudWithNonFinite());

    auto ground_msg = waitForFuture(std::move(ground_future), std::chrono::milliseconds(750));
    auto nonground_msg = waitForFuture(std::move(nonground_future), std::chrono::milliseconds(750));

    REQUIRE(ground_msg.has_value());
    REQUIRE(nonground_msg.has_value());
    REQUIRE(ground_msg->width * ground_msg->height + nonground_msg->width * nonground_msg->height > 0);
    REQUIRE(hasOnlyFinitePoints(*ground_msg));
    REQUIRE(hasOnlyFinitePoints(*nonground_msg));
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  node->deactivate();
  node->cleanup();
}

TEST_CASE_METHOD(
  wato::test::TestExecutorFixture, "Classifies ground vs elevated obstacles", "[node][integration][fast]")
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("sensor_height", 1.5);
  auto node = std::make_shared<GroundRemovalNode>(options);
  add_node(node);

  auto ground_sub =
    std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kGroundTopic);
  auto nonground_sub =
    std::make_shared<wato::test::SubscriberTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kNonGroundTopic);
  add_node(ground_sub);
  add_node(nonground_sub);

  auto input_pub =
    std::make_shared<wato::test::PublisherTestNode<sensor_msgs::msg::PointCloud2>>(GroundRemovalNode::kCloudTopic);
  add_node(input_pub);

  node->configure();
  node->activate();
  start_spinning();

  REQUIRE(waitForCondition(
    [&]() {
      return input_pub->count_subscribers(GroundRemovalNode::kCloudTopic) > 0 &&
             ground_sub->count_publishers(GroundRemovalNode::kGroundTopic) > 0 &&
             nonground_sub->count_publishers(GroundRemovalNode::kNonGroundTopic) > 0;
    },
    std::chrono::milliseconds(750),
    std::chrono::milliseconds(25)));

  std::vector<std::array<float, 3>> samples;
  // Ground points beyond typical min_range
  for (int i = 0; i < 20; ++i) {
    for (int j = -2; j <= 2; ++j) {
      const float x = 3.0f + static_cast<float>(i) * 0.3f;
      const float y = static_cast<float>(j) * 0.4f;
      samples.push_back({x, y, -1.5f});
    }
  }
  const int ground_expected = static_cast<int>(samples.size());

  // Elevated obstacles sharing similar ranges
  for (int i = 0; i < 10; ++i) {
    const float x = 3.5f + static_cast<float>(i) * 0.6f;
    const float y = (i % 2 == 0) ? 0.8f : -0.8f;
    samples.push_back({x, y, -0.5f});
  }
  const int obstacle_expected = static_cast<int>(samples.size()) - ground_expected;

  auto test_cloud = makePointCloudFromTriples(samples);

  auto ground_future = ground_sub->expect_next_message();
  auto nonground_future = nonground_sub->expect_next_message();

  input_pub->publish(test_cloud);

  auto ground_msg = waitForFuture(std::move(ground_future), std::chrono::milliseconds(1000));
  auto nonground_msg = waitForFuture(std::move(nonground_future), std::chrono::milliseconds(1000));

  REQUIRE(ground_msg.has_value());
  REQUIRE(nonground_msg.has_value());

  const auto ground_count = static_cast<int>(ground_msg->width * ground_msg->height);
  const auto nonground_count = static_cast<int>(nonground_msg->width * nonground_msg->height);
  const auto total_in = static_cast<int>(test_cloud.width * test_cloud.height);

  REQUIRE(ground_count + nonground_count <= total_in);
  REQUIRE(ground_count > 0);
  REQUIRE(nonground_count > 0);
  REQUIRE(ground_count >= static_cast<int>(0.5F * static_cast<float>(ground_expected)));
  REQUIRE(nonground_count >= static_cast<int>(0.5F * static_cast<float>(obstacle_expected)));

  node->deactivate();
  node->cleanup();
}
