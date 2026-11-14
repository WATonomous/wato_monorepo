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
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "test_fixtures/test_executor_fixture.hpp"

namespace wato::test
{

template <typename BaseFixture>
class ObservedExecutorFixture : public BaseFixture
{
public:
  rclcpp::Executor & executor()
  {
    return this->executor_;
  }
};

class ContextCheckingLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ContextCheckingLifecycleNode()
  : LifecycleNode("teardown_lifecycle_node")
  {}

  bool deactivated_with_valid_context = false;

protected:
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    deactivated_with_valid_context = rclcpp::ok();
    return CallbackReturn::SUCCESS;
  }
};

TEMPLATE_TEST_CASE(
  "Executor teardown keeps ROS valid for in-flight callbacks",
  "[executor_teardown]",
  TestExecutorFixture,
  MultiThreadedTestFixture)
{
  auto fixture = std::make_unique<ObservedExecutorFixture<TestType>>();
  auto node = std::make_shared<rclcpp::Node>("teardown_callback_node");
  fixture->add_node(node);

  auto started = std::make_shared<std::promise<void>>();
  auto started_future = started->get_future();
  auto finished = std::make_shared<std::promise<void>>();
  auto finished_future = finished->get_future();
  auto once = std::make_shared<std::once_flag>();
  auto * executor = &fixture->executor();

  auto timer = node->create_wall_timer(std::chrono::milliseconds(1), [node, started, finished, once, executor]() {
    std::call_once(*once, [&]() {
      started->set_value();
      // Hold this callback until teardown has canceled the executor. The
      // deadline also lets a failed test unwind without hanging its destructor.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (executor->is_spinning() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      try {
        if (executor->is_spinning()) {
          throw std::runtime_error("Executor was not canceled during teardown");
        }
        // This reproduces the joystick callback creating a timer as the test
        // ends. The old shutdown-before-join order invalidated the context here.
        auto next_timer = node->create_wall_timer(std::chrono::hours(1), []() {});
        next_timer->cancel();
        finished->set_value();
      } catch (...) {
        finished->set_exception(std::current_exception());
      }
    });
  });

  fixture->start_spinning();
  REQUIRE(started_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
  fixture.reset();

  REQUIRE(finished_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
  REQUIRE_NOTHROW(finished_future.get());
  REQUIRE_FALSE(rclcpp::ok());
}

TEMPLATE_TEST_CASE(
  "Executor teardown deactivates lifecycle nodes before ROS shutdown",
  "[executor_teardown]",
  TestExecutorFixture,
  MultiThreadedTestFixture)
{
  auto fixture = std::make_unique<TestType>();
  auto node = std::make_shared<ContextCheckingLifecycleNode>();
  fixture->add_node(node);
  REQUIRE(node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE(node->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  fixture.reset();

  REQUIRE(node->deactivated_with_valid_context);
  REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  REQUIRE_FALSE(rclcpp::ok());
}

TEMPLATE_TEST_CASE(
  "Executor can be stopped immediately after startup",
  "[executor_teardown]",
  TestExecutorFixture,
  MultiThreadedTestFixture)
{
  for (int i = 0; i < 20; ++i) {
    auto fixture = std::make_unique<TestType>();
    fixture->start_spinning();
    fixture.reset();
    REQUIRE_FALSE(rclcpp::ok());
  }
}

}  // namespace wato::test
