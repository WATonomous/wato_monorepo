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
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace wato::test
{

/**
 * @brief Client Test Node
 *
 * Templated service client test node providing
 * future-based synchronization for asynchronous service calls.
 *
 * @tparam ServiceType The ROS service type to call
 */
template <typename ServiceType>
class ClientTestNode : public rclcpp::Node
{
public:
  using RequestType = typename ServiceType::Request;
  using ResponseType = typename ServiceType::Response;

  /**
   * @brief ClientTestNode Constructor
   *
   * @param service_name The ROS service name to call
   * @param node_name The name for this ROS node
   * @param qos_profile QOS profile to use
   */
  explicit ClientTestNode(
    const std::string & service_name,
    const std::string & node_name = "client_test_node",
    const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS());

  /**
   * @brief Get the last response received
   *
   * @return std::optional<ResponseType> The most recently received response, or empty if none
   */
  std::optional<ResponseType> get_last_response() const;

  /**
   * @brief Send an asynchronous request and return future for response
   *
   * Sends async service request. Returns a future for service response.
   * Future will resolve with the response when the service call completes.
   *
   * @param request The request to send
   * @return std::future<ResponseType> Future that resolves with the service response
   */
  std::future<ResponseType> send_request_async(const RequestType & request);

  /**
   * @brief Reset internal state for a new test
   *
   * Clears the last response and resets state. Should be called between
   * test sections to ensure clean state.
   */
  void reset_for_new_test();

private:
  typename rclcpp::Client<ServiceType>::SharedPtr client_;

  std::mutex response_mutex_;
  std::optional<ResponseType> last_response_;
};

// Template implementation
template <typename ServiceType>
ClientTestNode<ServiceType>::ClientTestNode(
  const std::string & service_name, const std::string & node_name, const rclcpp::QoS & qos_profile)
: Node(node_name)
{
  client_ = this->create_client<ServiceType>(service_name, qos_profile.get_rmw_qos_profile());
  RCLCPP_INFO(this->get_logger(), "Created client for service: %s", service_name.c_str());
}

template <typename ServiceType>
std::optional<typename ClientTestNode<ServiceType>::ResponseType> ClientTestNode<ServiceType>::get_last_response() const
{
  std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(response_mutex_));
  return last_response_;
}

template <typename ServiceType>
std::future<typename ServiceType::Response> ClientTestNode<ServiceType>::send_request_async(const RequestType & request)
{
  RCLCPP_INFO(this->get_logger(), "Sending async request");

  auto promise = std::make_shared<std::promise<ResponseType>>();
  auto future = promise->get_future();

  if (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_WARN(this->get_logger(), "Service not available");
    promise->set_exception(std::make_exception_ptr(std::runtime_error("Service not available")));
    return future;
  }

  RCLCPP_INFO(this->get_logger(), "Service available, sending async request");
  auto request_ptr = std::make_shared<RequestType>(request);

  // Send request and set up callback to update last_response and resolve promise
  auto response_future = client_->async_send_request(
    request_ptr, [this, promise](typename rclcpp::Client<ServiceType>::SharedFuture future) {
      try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Received async response");

        {
          std::lock_guard<std::mutex> lock(response_mutex_);
          last_response_ = *response;
        }

        promise->set_value(*response);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Async request failed: %s", e.what());
        promise->set_exception(std::current_exception());
      }
    });

  return future;
}

template <typename ServiceType>
void ClientTestNode<ServiceType>::reset_for_new_test()
{
  std::lock_guard<std::mutex> lock(response_mutex_);
  last_response_.reset();
}

}  // namespace wato::test
