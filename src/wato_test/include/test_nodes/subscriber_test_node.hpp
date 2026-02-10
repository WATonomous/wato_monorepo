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

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace wato::test
{

/**
 * @brief Subscriber Test Node
 *
 * Templated subscription test node providing
 * message futures to facilitate event-driven testing.
 *
 * @tparam MessageType The ROS message type to subscribe to
 */
template <typename MessageType>
class SubscriberTestNode : public rclcpp::Node
{
public:
  /**
   * @brief SubscriberTestNode Constructor
   *
   * @param topic_name The ROS topic name to subscribe to
   * @param node_name The name for this ROS node
   * @param qos_profile QOS profile to use
   */
  explicit SubscriberTestNode(
    const std::string & topic_name,
    const std::string & node_name = "subscriber_test_node",
    const rclcpp::QoS & qos_profile = rclcpp::SystemDefaultsQoS());

  /**
   * @brief Get the last received message
   *
   * @return std::optional<MessageType> The most recently received message, or empty if none
   */
  std::optional<MessageType> get_last_message() const;

  /**
   * @brief Get the total number of messages received
   *
   * @return size_t The total count of messages received since creation or last reset
   */
  size_t get_message_count() const;

  /**
   * @brief Expect a future message
   *
   * Returns a future that resolves when a message is received
   *
   * @return std::future<MessageType>
   */
  std::future<MessageType> expect_next_message();

  /**
   * @brief Reset internal state for a new test
   *
   * Clears the last message and any pending futures.
   */
  void reset_for_new_test();

  /**
   * @brief Wait until this subscriber has at least the specified number of publishers
   *
   * @param count Minimum number of matched publishers to wait for
   * @param timeout Maximum time to wait
   * @return true if the required publisher count was reached, false on timeout
   */
  bool wait_for_publishers(
    size_t count = 1,
    std::chrono::milliseconds timeout = std::chrono::seconds(10)) const;

private:
  void message_callback(typename MessageType::UniquePtr msg);

  typename rclcpp::Subscription<MessageType>::SharedPtr subscription_;

  mutable std::mutex data_mutex_;
  std::optional<MessageType> last_message_;
  size_t message_count_ = 0;

  // Future-based synchronization
  std::queue<std::promise<MessageType>> message_promises_;
};

// Template implementation
template <typename MessageType>
SubscriberTestNode<MessageType>::SubscriberTestNode(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos_profile)
: Node(node_name)
{
  subscription_ = this->create_subscription<MessageType>(
    topic_name,
    qos_profile,
    std::bind(&SubscriberTestNode<MessageType>::message_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Created subscriber on topic: %s", topic_name.c_str());
}

template <typename MessageType>
std::optional<MessageType> SubscriberTestNode<MessageType>::get_last_message() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_message_;
}

template <typename MessageType>
size_t SubscriberTestNode<MessageType>::get_message_count() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return message_count_;
}

template <typename MessageType>
std::future<MessageType> SubscriberTestNode<MessageType>::expect_next_message()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  message_promises_.emplace();
  return message_promises_.back().get_future();
}

template <typename MessageType>
void SubscriberTestNode<MessageType>::reset_for_new_test()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_message_.reset();
  message_count_ = 0;

  // Clear any pending promises
  while (!message_promises_.empty()) {
    message_promises_.pop();
  }
}

template <typename MessageType>
void SubscriberTestNode<MessageType>::message_callback(typename MessageType::UniquePtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_message_ = *msg;
  message_count_++;

  // Resolve any waiting futures
  if (!message_promises_.empty()) {
    message_promises_.front().set_value(*msg);
    message_promises_.pop();
  }

  RCLCPP_DEBUG(this->get_logger(), "Received message");
}

template <typename MessageType>
bool SubscriberTestNode<MessageType>::wait_for_publishers(size_t count, std::chrono::milliseconds timeout) const
{
  auto start = std::chrono::steady_clock::now();
  while (subscription_->get_publisher_count() < count) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

}  // namespace wato::test
