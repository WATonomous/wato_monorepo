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

/**
 * @brief Monitors configured ROS 2 topics for liveness and message rate.
 *
 * Subscribes to an arbitrary list of topics (specified via the `topics` parameter)
 * using generic serialized-message subscriptions so no compile-time type knowledge
 * is required. For each topic the node tracks whether it exists in the ROS graph,
 * whether it has active publishers, and the measured message rate (Hz) over a
 * rolling window. Results are exposed as JSON via a lightweight HTTP server so that
 * external tools (e.g. the log viewer) can poll topic health without ROS access.
 *
 * Status values per topic:
 *  - **healthy**        – subscribed and receiving messages within the stale timeout
 *  - **stale**          – was receiving messages but none arrived within `stale_timeout` seconds
 *  - **no_publishers**  – topic exists in the graph but has zero publishers
 *  - **not_found**      – topic is not present in the ROS graph
 *
 * Parameters (loaded from YAML):
 *  - `topics`       (string[]) – list of topic names to monitor
 *  - `http_port`    (int)      – port for the HTTP health endpoint (default 8080)
 *  - `stale_timeout` (double)  – seconds with no messages before marking stale (default 5.0)
 *  - `window_size`  (int)      – rolling timestamp window size for rate calc (default 50)
 *  - `check_period` (double)   – seconds between graph-refresh cycles (default 2.0)
 */
class TopicHealthchecker : public rclcpp::Node
{
public:
  explicit TopicHealthchecker(const rclcpp::NodeOptions & options);
  ~TopicHealthchecker() override;

private:
  /// Per-topic monitoring state.
  struct TopicState
  {
    std::string name;  ///< Fully-qualified topic name.
    std::string status = "not_found";  ///< Current health status.
    std::string msg_type;  ///< Message type string (e.g. "sensor_msgs/msg/Image").
    double rate_hz = 0.0;  ///< Measured publish rate from rolling window.
    std::deque<std::chrono::steady_clock::time_point> timestamps;  ///< Rolling message arrival times.
    std::chrono::steady_clock::time_point last_received;  ///< Time of most recent message.
    rclcpp::GenericSubscription::SharedPtr subscription;  ///< Generic subscription handle.
    bool ever_received = false;  ///< True once at least one message has arrived.
  };

  /**
   * @brief Timer callback that discovers topics in the ROS graph and manages subscriptions.
   *
   * For each configured topic: checks graph presence and publisher count, creates a
   * generic subscription if publishers exist, and marks stale topics.
   */
  void refresh_subscriptions();

  /**
   * @brief Subscription callback invoked on every received message.
   * @param topic_name The topic that received a message.
   *
   * Records the arrival timestamp, updates the rolling window, and recomputes
   * the message rate.
   */
  void on_message(const std::string & topic_name);

  /**
   * @brief Serializes the current health state of all topics to a JSON string.
   * @return JSON string suitable for the HTTP /health response.
   */
  std::string build_health_json() const;

  /**
   * @brief Runs a minimal HTTP server in a background thread.
   *
   * Serves GET /health with the JSON health payload. Uses POSIX sockets
   * so no external HTTP library is required.
   */
  void run_http_server();

  std::vector<std::string> expected_topics_;  ///< Topics to monitor (from config).
  std::unordered_map<std::string, TopicState> topic_states_;  ///< Per-topic state map.
  mutable std::mutex mutex_;  ///< Guards topic_states_ for thread-safe HTTP reads.

  int http_port_;  ///< Port for the HTTP health server.
  double stale_timeout_;  ///< Seconds before a silent topic is marked stale.
  size_t window_size_;  ///< Max timestamps kept per topic for rate calculation.

  rclcpp::TimerBase::SharedPtr timer_;  ///< Periodic graph-refresh timer.
  std::thread http_thread_;  ///< Background thread running the HTTP server.
  int server_fd_{-1};  ///< HTTP server socket file descriptor.
  std::atomic<bool> running_{true};  ///< Signals the HTTP thread to stop on destruction.
};

}  // namespace topic_healthchecker
