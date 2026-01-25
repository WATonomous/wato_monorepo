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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <wato_test/wato_test.hpp>

#include "carla_lifecycle/lifecycle_manager_node.hpp"

namespace carla_lifecycle
{

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

/**
 * @brief Mock Lifecycle Node that simulates lifecycle state machine behavior.
 *
 * Can be created with a namespace to test namespace handling.
 */
class MockLifecycleNode : public rclcpp::Node
{
public:
  explicit MockLifecycleNode(const std::string & node_name, const std::string & ns = "")
  : Node(
      node_name,
      rclcpp::NodeOptions().arguments(
        ns.empty() ? std::vector<std::string>{} : std::vector<std::string>{"--ros-args", "-r", "__ns:=/" + ns}))
  , current_state_(State::PRIMARY_STATE_UNCONFIGURED)
  {
    // Build the service prefix based on namespace
    std::string service_prefix = ns.empty() ? "/" + node_name : "/" + ns + "/" + node_name;

    change_state_srv_ = this->create_service<lifecycle_msgs::srv::ChangeState>(
      service_prefix + "/change_state",
      std::bind(&MockLifecycleNode::handleChangeState, this, std::placeholders::_1, std::placeholders::_2));

    get_state_srv_ = this->create_service<lifecycle_msgs::srv::GetState>(
      service_prefix + "/get_state",
      std::bind(&MockLifecycleNode::handleGetState, this, std::placeholders::_1, std::placeholders::_2));
  }

  uint8_t getCurrentState() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
  }

private:
  void handleChangeState(
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    bool valid_transition = false;
    switch (request->transition.id) {
      case Transition::TRANSITION_CONFIGURE:
        if (current_state_ == State::PRIMARY_STATE_UNCONFIGURED) {
          current_state_ = State::PRIMARY_STATE_INACTIVE;
          valid_transition = true;
        }
        break;
      case Transition::TRANSITION_ACTIVATE:
        if (current_state_ == State::PRIMARY_STATE_INACTIVE) {
          current_state_ = State::PRIMARY_STATE_ACTIVE;
          valid_transition = true;
        }
        break;
      case Transition::TRANSITION_DEACTIVATE:
        if (current_state_ == State::PRIMARY_STATE_ACTIVE) {
          current_state_ = State::PRIMARY_STATE_INACTIVE;
          valid_transition = true;
        }
        break;
      case Transition::TRANSITION_CLEANUP:
        if (current_state_ == State::PRIMARY_STATE_INACTIVE) {
          current_state_ = State::PRIMARY_STATE_UNCONFIGURED;
          valid_transition = true;
        }
        break;
      default:
        valid_transition = false;
    }

    response->success = valid_transition;
  }

  void handleGetState(
    const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> /*request*/,
    std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    response->current_state.id = current_state_;
  }

  mutable std::mutex state_mutex_;
  uint8_t current_state_;
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_srv_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_srv_;
};

TEST_CASE_METHOD(
  wato::test::MultiThreadedTestFixture,
  "LifecycleManagerNode handles relative node names",
  "[lifecycle_manager][node_names]")
{
  // Create a mock node without namespace
  auto mock_node = std::make_shared<MockLifecycleNode>("test_node");
  add_node(mock_node);

  // Create lifecycle manager with relative node name (no leading /)
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("node_names", std::vector<std::string>{"test_node"}),
    rclcpp::Parameter("autostart", false),
    rclcpp::Parameter("scenario_server_name", "scenario_server"),
    rclcpp::Parameter("service_timeout", 5.0),
  });

  auto manager = std::make_shared<LifecycleManagerNode>(options);
  add_node(manager);

  start_spinning();

  // Give time for service discovery
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  SECTION("Manager can find node with relative name")
  {
    REQUIRE(mock_node->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_CASE_METHOD(
  wato::test::MultiThreadedTestFixture,
  "LifecycleManagerNode handles absolute node names",
  "[lifecycle_manager][node_names]")
{
  // Create a mock node without namespace
  auto mock_node = std::make_shared<MockLifecycleNode>("test_node");
  add_node(mock_node);

  // Create lifecycle manager with absolute node name (with leading /)
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("node_names", std::vector<std::string>{"/test_node"}),
    rclcpp::Parameter("autostart", false),
    rclcpp::Parameter("scenario_server_name", "scenario_server"),
    rclcpp::Parameter("service_timeout", 5.0),
  });

  auto manager = std::make_shared<LifecycleManagerNode>(options);
  add_node(manager);

  start_spinning();

  // Give time for service discovery
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  SECTION("Manager can find node with absolute name")
  {
    REQUIRE(mock_node->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_CASE_METHOD(
  wato::test::MultiThreadedTestFixture,
  "LifecycleManagerNode handles namespaced node names",
  "[lifecycle_manager][node_names][namespace]")
{
  // Create a mock node in the "carla" namespace
  auto mock_node = std::make_shared<MockLifecycleNode>("lidar_publisher", "carla");
  add_node(mock_node);

  SECTION("Manager finds node with relative namespaced name")
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"carla/lidar_publisher"}),
      rclcpp::Parameter("autostart", false),
      rclcpp::Parameter("scenario_server_name", "scenario_server"),
      rclcpp::Parameter("service_timeout", 5.0),
    });

    auto manager = std::make_shared<LifecycleManagerNode>(options);
    add_node(manager);

    start_spinning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    REQUIRE(mock_node->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Manager finds node with absolute namespaced name")
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"/carla/lidar_publisher"}),
      rclcpp::Parameter("autostart", false),
      rclcpp::Parameter("scenario_server_name", "scenario_server"),
      rclcpp::Parameter("service_timeout", 5.0),
    });

    auto manager = std::make_shared<LifecycleManagerNode>(options);
    add_node(manager);

    start_spinning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    REQUIRE(mock_node->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_CASE_METHOD(
  wato::test::MultiThreadedTestFixture,
  "LifecycleManagerNode handles mixed node name formats",
  "[lifecycle_manager][node_names]")
{
  // Create nodes with different namespace configurations
  auto node1 = std::make_shared<MockLifecycleNode>("node_a");
  auto node2 = std::make_shared<MockLifecycleNode>("node_b", "carla");
  auto node3 = std::make_shared<MockLifecycleNode>("node_c", "carla");

  add_node(node1);
  add_node(node2);
  add_node(node3);

  // Mix of relative, absolute, and namespaced names
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "node_names",
      std::vector<std::string>{
        "node_a",  // relative, no namespace
        "/carla/node_b",  // absolute, with namespace
        "carla/node_c",  // relative, with namespace
      }),
    rclcpp::Parameter("autostart", false),
    rclcpp::Parameter("scenario_server_name", "scenario_server"),
    rclcpp::Parameter("service_timeout", 5.0),
  });

  auto manager = std::make_shared<LifecycleManagerNode>(options);
  add_node(manager);

  start_spinning();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  SECTION("All nodes are accessible regardless of name format")
  {
    REQUIRE(node1->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(node2->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(node3->getCurrentState() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}

}  // namespace carla_lifecycle
