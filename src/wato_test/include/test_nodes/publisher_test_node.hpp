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
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace wato::test
{

/**
 * @brief Publisher Test Node
 *
 * Simple templated publisher test node.
 *
 * @tparam MessageType The ROS message type to publish
 */
template <typename MessageType>
class PublisherTestNode : public rclcpp::Node
{
public:
  /**
   * @brief PublisherTestNode Constructor
   *
   * @param topic_name The ROS topic name to publish to
   * @param node_name The name for this ROS node
   * @param qos_profile QOS profile to use
   */
  explicit PublisherTestNode(
    const std::string & topic_name,
    const std::string & node_name = "publisher_test_node",
    const rclcpp::QoS & qos_profile = rclcpp::SystemDefaultsQoS());

  /**
   * @brief Publish a message to the topic
   *
   * @param message The message to publish
   */
  void publish(const MessageType & message);

private:
  typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};

// Template implementation
template <typename MessageType>
PublisherTestNode<MessageType>::PublisherTestNode(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos_profile)
: Node(node_name)
{
  publisher_ = this->create_publisher<MessageType>(topic_name, qos_profile);
  RCLCPP_INFO(this->get_logger(), "Created publisher on topic: %s", topic_name.c_str());
}

template <typename MessageType>
void PublisherTestNode<MessageType>::publish(const MessageType & message)
{
  publisher_->publish(message);
  RCLCPP_DEBUG(this->get_logger(), "Published message");
}

}  // namespace wato::test
