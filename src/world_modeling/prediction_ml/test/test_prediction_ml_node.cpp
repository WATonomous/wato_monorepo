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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "prediction_ml/prediction_ml_node.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

namespace prediction_ml
{
TEST(PredictionMlNode, PublishesFallbackWhenMtrIsDisabled)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mtr.enabled", false)});
  auto node = std::make_shared<PredictionMlNode>(options);
  auto client = std::make_shared<rclcpp::Node>("prediction_ml_test_client");

  world_model_msgs::msg::WorldObjectArray received;
  bool got_output = false;
  auto output_sub = client->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "/world_object_seeds", 10, [&received, &got_output](world_model_msgs::msg::WorldObjectArray::ConstSharedPtr msg) {
      received = *msg;
      got_output = true;
    });
  auto tracks_pub = client->create_publisher<vision_msgs::msg::Detection3DArray>("/tracks_3d", 10);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);

  vision_msgs::msg::Detection3DArray detections;
  detections.header.frame_id = "map";
  detections.header.stamp.sec = 10;
  detections.detections.resize(1);
  detections.detections[0].id = "track-1";
  detections.detections[0].bbox.center.orientation.w = 1.0;
  detections.detections[0].bbox.size.x = 4.0;

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!got_output && std::chrono::steady_clock::now() < deadline) {
    tracks_pub->publish(detections);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(got_output);
  ASSERT_EQ(received.objects.size(), 1u);
  ASSERT_EQ(received.objects[0].predictions.size(), 1u);
  EXPECT_FALSE(received.objects[0].predictions[0].poses.empty());

  EXPECT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
}  // namespace prediction_ml
