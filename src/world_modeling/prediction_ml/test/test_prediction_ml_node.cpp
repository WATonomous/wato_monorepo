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

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "prediction_ml/prediction_ml_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

template <typename MessageT>
std::shared_ptr<MessageT> stamped(const std::int32_t seconds)
{
  auto message = std::make_shared<MessageT>();
  message->header.stamp.sec = seconds;
  return message;
}

}  // namespace

TEST(ExactStampJoiner, CompletesOutOfOrderOnceWithoutBorrowingStaleInputs)
{
  prediction_ml::ExactStampJoiner joiner;

  EXPECT_FALSE(joiner.addTracks(stamped<vision_msgs::msg::Detection3DArray>(10)));
  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(9)));
  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(10)));
  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(10)));
  auto complete = joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(10));

  ASSERT_TRUE(complete);
  EXPECT_EQ(complete->tracks->header.stamp.sec, 10);
  EXPECT_EQ(complete->pose->header.stamp.sec, 10);
  EXPECT_EQ(complete->odometry->header.stamp.sec, 10);
  EXPECT_EQ(complete->lanelet->header.stamp.sec, 10);

  EXPECT_FALSE(joiner.addTracks(stamped<vision_msgs::msg::Detection3DArray>(10)));
  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(10)));
  EXPECT_FALSE(joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(10)));
  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(10)));
}

TEST(ExactStampJoiner, MissingInputAndZeroStampNeverComplete)
{
  prediction_ml::ExactStampJoiner joiner;

  EXPECT_FALSE(joiner.addTracks(stamped<vision_msgs::msg::Detection3DArray>(20)));
  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(20)));
  EXPECT_FALSE(joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(20)));
  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(0)));
  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(21)));
}

TEST(ExactStampJoiner, EvictsOldestPendingMessageAtCapacity)
{
  prediction_ml::ExactStampJoiner joiner;
  for (std::int32_t stamp = 1; stamp <= 33; ++stamp) {
    EXPECT_FALSE(joiner.addTracks(stamped<vision_msgs::msg::Detection3DArray>(stamp)));
  }

  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(1)));
  EXPECT_FALSE(joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(1)));
  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(1)));

  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(2)));
  EXPECT_FALSE(joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(2)));
  EXPECT_TRUE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(2)));
}

TEST(ExactStampJoiner, ClearDropsEveryPartialTuple)
{
  prediction_ml::ExactStampJoiner joiner;
  EXPECT_FALSE(joiner.addTracks(stamped<vision_msgs::msg::Detection3DArray>(30)));
  EXPECT_FALSE(joiner.addPose(stamped<geometry_msgs::msg::PoseStamped>(30)));
  EXPECT_FALSE(joiner.addOdometry(stamped<nav_msgs::msg::Odometry>(30)));

  joiner.clear();

  EXPECT_FALSE(joiner.addLanelet(stamped<lanelet_msgs::msg::LaneletAhead>(30)));
}

TEST(PredictionMlNode, ConfiguresSeparateProductionAndMtrDiagnosticPublishers)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/world_modeling"});
  options.parameter_overrides(
    {rclcpp::Parameter("mtr.pinned_target_ids", std::vector<std::string>{"__ego__"})});
  auto node = std::make_shared<prediction_ml::PredictionMlNode>(options);
  ASSERT_EQ(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto production =
    node->get_publishers_info_by_topic("/world_modeling/world_object_seeds");
  const auto diagnostics =
    node->get_publishers_info_by_topic("/world_modeling/mtr_predictions");
  EXPECT_EQ(production.size(), 1U);
  EXPECT_EQ(diagnostics.size(), 1U);
  ASSERT_EQ(diagnostics.front().topic_type(), "world_model_msgs/msg/WorldObjectArray");

  EXPECT_EQ(
    node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    node->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    node->cleanup().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  node.reset();
  rclcpp::shutdown();
}
