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

#include <chrono>
#include <exception>
#include <future>
#include <stdexcept>
#include <thread>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>

namespace wato::test
{

namespace
{

// Starts executor.spin() on a background thread and returns once it is
// spinning, so a later cancel() cannot be lost. Throws if spin() exits first.
template <typename ExecutorT>
void launch_spin_thread(ExecutorT & executor, std::thread & thread, std::future<void> & done)
{
  std::packaged_task<void()> task([&executor]() { executor.spin(); });
  done = task.get_future();
  thread = std::thread(std::move(task));

  while (!executor.is_spinning()) {
    if (done.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      done.get();  // Rethrows the exception from spin(), if any.
      throw std::runtime_error("Executor stopped before it started spinning (is the ROS context shut down?)");
    }
    std::this_thread::yield();
  }
}

// Cancels the executor and joins its thread while the context is still valid.
// Blocking callbacks must use bounded waits so cancellation can finish.
template <typename ExecutorT>
void stop_spin_thread(ExecutorT & executor, std::thread & thread, std::future<void> & done, const char * logger)
{
  executor.cancel();
  if (!thread.joinable()) {
    return;
  }
  thread.join();

  if (done.valid()) {
    try {
      done.get();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger(logger), "Executor threw while spinning: %s", e.what());
    }
  }
}

}  // namespace

ROS2Initializer::ROS2Initializer()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
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
  // Keep the context valid until in-flight callbacks finish. In particular,
  // callbacks may still publish or create timers after the test body returns.
  stop_spin_thread(executor_, spin_thread_, spin_done_, "TestExecutorFixture");

  for (auto & node_base : nodes_) {
    if (node_base) {
      executor_.remove_node(node_base);
    }
  }
  nodes_.clear();

  for (auto & node : lifecycle_nodes_) {
    if (node && node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      try {
        node->deactivate();
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("TestExecutorFixture"), "Failed to deactivate node during cleanup: %s", e.what());
      }
    }
  }

  lifecycle_nodes_.clear();
  managed_nodes_.clear();
}

void TestExecutorFixture::start_spinning()
{
  if (!spin_thread_.joinable()) {
    launch_spin_thread(executor_, spin_thread_, spin_done_);
  }
}

MultiThreadedTestFixture::MultiThreadedTestFixture()
: ROS2Initializer()
{}

MultiThreadedTestFixture::~MultiThreadedTestFixture()
{
  // Join all callback threads before ROS2Initializer shuts down the context.
  stop_spin_thread(executor_, spin_thread_, spin_done_, "MultiThreadedTestFixture");

  for (auto & node_base : nodes_) {
    if (node_base) {
      executor_.remove_node(node_base);
    }
  }
  nodes_.clear();

  for (auto & node : lifecycle_nodes_) {
    if (node && node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      try {
        node->deactivate();
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("MultiThreadedTestFixture"), "Failed to deactivate node during cleanup: %s", e.what());
      }
    }
  }

  lifecycle_nodes_.clear();
  managed_nodes_.clear();
}

void MultiThreadedTestFixture::start_spinning()
{
  if (!spin_thread_.joinable()) {
    launch_spin_thread(executor_, spin_thread_, spin_done_);
  }
}

}  // namespace wato::test
