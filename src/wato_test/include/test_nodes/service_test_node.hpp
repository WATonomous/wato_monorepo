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

#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace wato::test
{

/**
 * @brief Service Test Node
 *
 * Templated service server test node providing
 * future-based synchronization for deterministic testing.
 *
 * @tparam ServiceType The ROS service type to provide
 */
template <typename ServiceType>
class ServiceTestNode : public rclcpp::Node
{
public:
  using RequestType = typename ServiceType::Request;
  using ResponseType = typename ServiceType::Response;

  /**
   * @brief ServiceTestNode Constructor
   *
   * @param service_name The ROS service name to provide
   * @param node_name The name for this ROS node
   * @param qos_profile QOS profile to use
   */
  explicit ServiceTestNode(
    const std::string & service_name,
    const std::string & node_name = "service_test_node",
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  /**
   * @brief Set the response that the service will return
   *
   * @param response The response to send for service requests
   */
  void set_response(const ResponseType & response);

  /**
   * @brief Get a future that resolves when the service is called
   *
   * @return std::future<RequestType> Future that resolves with the service request
   */
  std::future<RequestType> expect_service_call();

  /**
   * @brief Reset internal state for a new test
   *
   * Clears any pending futures.
   */
  void reset_for_new_test();

private:
  void service_callback(const typename RequestType::SharedPtr request, typename ResponseType::SharedPtr response);

  typename rclcpp::Service<ServiceType>::SharedPtr service_;

  std::mutex response_mutex_;
  std::optional<ResponseType> stored_response_;

  // Future-based synchronization
  std::queue<std::promise<RequestType>> request_promises_;
};

// Template implementation
template <typename ServiceType>
ServiceTestNode<ServiceType>::ServiceTestNode(
  const std::string & service_name, const std::string & node_name, const rclcpp::QoS & qos_profile)
: Node(node_name)
{
  service_ = this->create_service<ServiceType>(
    service_name,
    std::bind(&ServiceTestNode<ServiceType>::service_callback, this, std::placeholders::_1, std::placeholders::_2),
    qos_profile.get_rmw_qos_profile());

  RCLCPP_INFO(this->get_logger(), "Created service: %s", service_name.c_str());
}

template <typename ServiceType>
void ServiceTestNode<ServiceType>::set_response(const ResponseType & response)
{
  std::lock_guard<std::mutex> lock(response_mutex_);
  stored_response_ = response;
}

template <typename ServiceType>
void ServiceTestNode<ServiceType>::service_callback(
  const typename RequestType::SharedPtr request, typename ResponseType::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(response_mutex_);

  // Resolve request future
  if (!request_promises_.empty()) {
    request_promises_.front().set_value(*request);
    request_promises_.pop();
  }

  if (stored_response_.has_value()) {
    *response = stored_response_.value();
  }

  RCLCPP_DEBUG(this->get_logger(), "Service request handled");
}

template <typename ServiceType>
std::future<typename ServiceType::Request> ServiceTestNode<ServiceType>::expect_service_call()
{
  std::lock_guard<std::mutex> lock(response_mutex_);
  request_promises_.emplace();
  return request_promises_.back().get_future();
}

template <typename ServiceType>
void ServiceTestNode<ServiceType>::reset_for_new_test()
{
  std::lock_guard<std::mutex> lock(response_mutex_);
  stored_response_.reset();

  // Clear any pending promises
  while (!request_promises_.empty()) {
    request_promises_.pop();
  }
}

}  // namespace wato::test
