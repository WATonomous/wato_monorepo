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

#pragma once

#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace wato::test
{

/**
 * @brief ROS2 Initializer base class
 *
 * Used as a private base class to control initialization order,
 * and ensures ROS2 is initialized before any derived class members are constructed.
 */
struct ROS2Initializer
{
  ROS2Initializer();
  ~ROS2Initializer();
};

/**
 * @brief Test Executor Fixture
 *
 * Catch2 test fixture that manages a single executor and thread for spinning
 * multiple test nodes.
 */
class TestExecutorFixture : private ROS2Initializer
{
public:
  /**
   * @brief Constructor - initializes ROS and creates executor
   */
  TestExecutorFixture();

  /**
   * @brief Destructor - stops executor and shuts down ROS
   */
  ~TestExecutorFixture();

  /**
   * @brief Start spinning the executor in a background thread
   * Call this after adding all nodes to the executor
   */
  void start_spinning();

  /**
   * @brief Add a templated test node to be spun by this executor
   *
   * @tparam T The test node type (must inherit from rclcpp::Node or rclcpp_lifecycle::LifecycleNode)
   * @param node The test node to add
   */
  template <typename T>
  void add_node(std::shared_ptr<T> node)
  {
    static_assert(
      std::is_base_of_v<rclcpp::Node, T> || std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, T>,
      "T must inherit from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

    executor_.add_node(node->get_node_base_interface());

    // Track lifecycle nodes for cleanup
    if constexpr (std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, T>) {
      lifecycle_nodes_.push_back(std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node));
    }
  }

protected:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> lifecycle_nodes_;
};

}  // namespace wato::test
