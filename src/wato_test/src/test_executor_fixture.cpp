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

#include "test_fixtures/test_executor_fixture.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace wato::test
{

ROS2Initializer::ROS2Initializer()
{
  rclcpp::init(0, nullptr);
}

ROS2Initializer::~ROS2Initializer()
{
  rclcpp::shutdown();
}

TestExecutorFixture::TestExecutorFixture()
: ROS2Initializer()
{}

TestExecutorFixture::~TestExecutorFixture()
{
  // Deactivate all lifecycle nodes to ensure proper cleanup
  for (auto & node : lifecycle_nodes_) {
    if (node && node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      try {
        node->deactivate();
      } catch (const std::exception & e) {
        // Log but don't throw during destruction
        RCLCPP_WARN(
          rclcpp::get_logger("TestExecutorFixture"), "Failed to deactivate node during cleanup: %s", e.what());
      }
    }
  }

  // Cancel executor and wait for thread to finish
  executor_.cancel();

  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void TestExecutorFixture::start_spinning()
{
  if (!spin_thread_.joinable()) {
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }
}

MultiThreadedTestFixture::MultiThreadedTestFixture()
: ROS2Initializer()
{}

MultiThreadedTestFixture::~MultiThreadedTestFixture()
{
  // Deactivate all lifecycle nodes to ensure proper cleanup
  for (auto & node : lifecycle_nodes_) {
    if (node && node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      try {
        node->deactivate();
      } catch (const std::exception & e) {
        // Log but don't throw during destruction
        RCLCPP_WARN(
          rclcpp::get_logger("MultiThreadedTestFixture"), "Failed to deactivate node during cleanup: %s", e.what());
      }
    }
  }

  // Cancel executor and wait for thread to finish
  executor_.cancel();

  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void MultiThreadedTestFixture::start_spinning()
{
  if (!spin_thread_.joinable()) {
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }
}

}  // namespace wato::test
