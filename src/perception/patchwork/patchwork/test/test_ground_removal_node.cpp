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
#include <future>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <wato_test/wato_test.hpp>

#include "patchworkpp/ground_removal_core.hpp"
#include "patchworkpp/ground_removal_node.hpp"

using wato::perception::patchworkpp::GroundRemovalCore;
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

// =============================================================================
// TEST 1: PointCloud2 to Eigen conversion
// WHY: This is the fundamental data transformation that all processing depends on.
//      If this fails, nothing else will work correctly.
// =============================================================================
TEST_CASE("PointCloud2 to Eigen conversion preserves point data", "[core][fast]")
{
  std::vector<std::array<float, 3>> points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(makePointCloudFromTriples(points));

  Eigen::MatrixX3f eigen_cloud = GroundRemovalCore::pointCloud2ToEigen(msg);

  REQUIRE(eigen_cloud.rows() == 2);
  REQUIRE(eigen_cloud.cols() == 3);
  REQUIRE_THAT(eigen_cloud(0, 0), Catch::Matchers::WithinRel(1.0f));
  REQUIRE_THAT(eigen_cloud(0, 1), Catch::Matchers::WithinRel(2.0f));
  REQUIRE_THAT(eigen_cloud(0, 2), Catch::Matchers::WithinRel(3.0f));
  REQUIRE_THAT(eigen_cloud(1, 0), Catch::Matchers::WithinRel(4.0f));
  REQUIRE_THAT(eigen_cloud(1, 1), Catch::Matchers::WithinRel(5.0f));
  REQUIRE_THAT(eigen_cloud(1, 2), Catch::Matchers::WithinRel(6.0f));
}

// =============================================================================
// TEST 2: Lifecycle state machine
// WHY: The node is a lifecycle node - it MUST transition through states correctly.
//      This verifies the node can be configured, activated, deactivated, and cleaned up.
// =============================================================================
TEST_CASE("Lifecycle node transitions through all states", "[node][fast]")
{
  RclcppGuard guard;
  auto node = std::make_shared<GroundRemovalNode>(rclcpp::NodeOptions{});

  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  REQUIRE(node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// =============================================================================
// TEST 3: End-to-end ground segmentation
// WHY: This is the PRIMARY purpose of the node - receive point cloud, segment it,
//      publish ground and non-ground clouds. Uses realistic synthetic data with
//      ground plane points and elevated obstacle points.
// =============================================================================
TEST_CASE_METHOD(
  wato::test::TestExecutorFixture, "Node segments point cloud into ground and non-ground", "[node][integration]")
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_frame", "base_link");
  auto node = std::make_shared<GroundRemovalNode>(options);
  add_node(node);

  // Publish a static transform: base_link -> test_frame with z=1.5 (sensor height)
  auto tf_node = std::make_shared<rclcpp::Node>("tf_broadcaster_test_node");
  add_node(tf_node);
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tf_node);
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = rclcpp::Clock().now();
  tf_msg.header.frame_id = "base_link";
  tf_msg.child_frame_id = "test_frame";
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 1.5;
  tf_msg.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_msg);

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

  // Create realistic test data: ground plane + obstacles
  std::vector<std::array<float, 3>> samples;

  // Ground points: flat plane at z = -1.5 (sensor height is 1.5, so ground is at -1.5)
  for (int i = 0; i < 20; ++i) {
    for (int j = -2; j <= 2; ++j) {
      const float x = 3.0f + static_cast<float>(i) * 0.3f;
      const float y = static_cast<float>(j) * 0.4f;
      samples.push_back({x, y, -1.5f});
    }
  }

  // Obstacle points: elevated above ground at z = -0.5
  for (int i = 0; i < 10; ++i) {
    const float x = 3.5f + static_cast<float>(i) * 0.6f;
    const float y = (i % 2 == 0) ? 0.8f : -0.8f;
    samples.push_back({x, y, -0.5f});
  }

  auto test_cloud = makePointCloudFromTriples(samples);

  auto ground_future = ground_sub->expect_next_message();
  auto nonground_future = nonground_sub->expect_next_message();

  input_pub->publish(test_cloud);

  auto ground_msg = waitForFuture(std::move(ground_future), std::chrono::milliseconds(1000));
  auto nonground_msg = waitForFuture(std::move(nonground_future), std::chrono::milliseconds(1000));

  // Verify outputs are published
  REQUIRE(ground_msg.has_value());
  REQUIRE(nonground_msg.has_value());

  // Verify header is preserved
  REQUIRE(ground_msg->header.frame_id == "test_frame");
  REQUIRE(nonground_msg->header.frame_id == "test_frame");

  // Verify algorithm produces reasonable segmentation
  const auto ground_count = static_cast<int>(ground_msg->width * ground_msg->height);
  const auto nonground_count = static_cast<int>(nonground_msg->width * nonground_msg->height);
  const auto total_in = static_cast<int>(test_cloud.width * test_cloud.height);

  REQUIRE(ground_count + nonground_count <= total_in);
  REQUIRE(ground_count > 0);
  REQUIRE(nonground_count > 0);
}
