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

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace topic_healthchecker
{

class TopicHealthchecker : public rclcpp::Node
{
public:
  explicit TopicHealthchecker(const rclcpp::NodeOptions & options);
  ~TopicHealthchecker() override;

private:
  struct TopicState
  {
    std::string name;
    std::string status = "not_found";  // healthy, stale, no_publishers, not_found
    std::string msg_type;
    double rate_hz = 0.0;
    std::deque<std::chrono::steady_clock::time_point> timestamps;
    std::chrono::steady_clock::time_point last_received;
    rclcpp::GenericSubscription::SharedPtr subscription;
    bool ever_received = false;
  };

  void refresh_subscriptions();
  void on_message(const std::string & topic_name);
  std::string build_health_json() const;
  void run_http_server();

  std::vector<std::string> expected_topics_;
  std::unordered_map<std::string, TopicState> topic_states_;
  mutable std::mutex mutex_;

  int http_port_;
  double stale_timeout_;
  size_t window_size_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::thread http_thread_;
  int server_fd_{-1};
  std::atomic<bool> running_{true};
};

}  // namespace topic_healthchecker
