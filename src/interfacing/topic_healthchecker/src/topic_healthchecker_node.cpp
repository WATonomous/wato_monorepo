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

#include "topic_healthchecker/topic_healthchecker_node.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace topic_healthchecker
{

TopicHealthchecker::TopicHealthchecker(const rclcpp::NodeOptions & options)
: Node("topic_healthchecker", options)
{
  this->declare_parameter<std::vector<std::string>>("topics");
  this->declare_parameter<int>("http_port");
  this->declare_parameter<double>("stale_timeout");
  this->declare_parameter<int>("window_size");
  this->declare_parameter<double>("check_period");

  expected_topics_ = this->get_parameter("topics").as_string_array();
  http_port_ = this->get_parameter("http_port").as_int();
  stale_timeout_ = this->get_parameter("stale_timeout").as_double();
  window_size_ = static_cast<size_t>(this->get_parameter("window_size").as_int());
  double check_period = this->get_parameter("check_period").as_double();

  // Initialize state for each configured topic
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & topic : expected_topics_) {
      TopicState state;
      state.name = topic;
      topic_states_[topic] = std::move(state);
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Topic healthchecker monitoring %zu topics, HTTP on port %d",
    expected_topics_.size(),
    http_port_);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(check_period), std::bind(&TopicHealthchecker::refresh_subscriptions, this));

  http_thread_ = std::thread(&TopicHealthchecker::run_http_server, this);
}

TopicHealthchecker::~TopicHealthchecker()
{
  running_.store(false);
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
  }
  if (http_thread_.joinable()) {
    http_thread_.join();
  }
}

void TopicHealthchecker::refresh_subscriptions()
{
  // Build map of currently active topics in the ROS graph
  auto graph_topics = this->get_topic_names_and_types();
  std::unordered_map<std::string, std::string> active_topics;
  for (const auto & [name, types] : graph_topics) {
    if (!types.empty()) {
      active_topics[name] = types[0];
    }
  }

  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & [topic_name, state] : topic_states_) {
    auto it = active_topics.find(topic_name);
    bool in_graph = (it != active_topics.end());

    if (!in_graph) {
      // Topic doesn't exist in the graph
      state.status = "not_found";
      state.msg_type.clear();
      state.subscription.reset();
      continue;
    }

    size_t pub_count = this->count_publishers(topic_name);
    state.msg_type = it->second;

    if (pub_count == 0) {
      state.status = "no_publishers";
      state.subscription.reset();
      continue;
    }

    // Topic exists with publishers -- create subscription if we don't have one
    if (!state.subscription) {
      try {
        state.subscription = this->create_generic_subscription(
          topic_name,
          it->second,
          rclcpp::SensorDataQoS(),
          [this, topic_name](std::shared_ptr<const rclcpp::SerializedMessage>) { this->on_message(topic_name); });
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s [%s]", topic_name.c_str(), it->second.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Failed to subscribe to %s: %s", topic_name.c_str(), e.what());
        state.status = "no_publishers";
        continue;
      }
    }

    // Check for staleness
    if (state.ever_received) {
      double age_s = std::chrono::duration<double>(now - state.last_received).count();
      if (age_s > stale_timeout_) {
        state.status = "stale";
        state.rate_hz = 0.0;
      }
      // else stays "healthy" (set by on_message)
    }
    // If never received but subscription exists, keep waiting (status stays as-is until first msg)
  }
}

void TopicHealthchecker::on_message(const std::string & topic_name)
{
  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_states_.find(topic_name);
  if (it == topic_states_.end()) {
    return;
  }

  auto & state = it->second;
  state.timestamps.push_back(now);
  while (state.timestamps.size() > window_size_) {
    state.timestamps.pop_front();
  }

  state.last_received = now;
  state.ever_received = true;
  state.status = "healthy";

  // Compute rate from rolling window
  if (state.timestamps.size() >= 2) {
    double dt = std::chrono::duration<double>(state.timestamps.back() - state.timestamps.front()).count();
    if (dt > 0.0) {
      state.rate_hz = static_cast<double>(state.timestamps.size() - 1) / dt;
    }
  } else {
    state.rate_hz = 0.0;
  }
}

static std::string escape_json_string(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out += '"';
  for (char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
    }
  }
  out += '"';
  return out;
}

std::string TopicHealthchecker::build_health_json() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = std::chrono::steady_clock::now();

  std::ostringstream ss;
  ss << std::fixed;
  ss << "{\"topics\":{";

  bool first = true;
  for (const auto & [name, state] : topic_states_) {
    if (!first) {
      ss << ",";
    }
    first = false;

    ss << escape_json_string(name) << ":{";
    ss << "\"status\":" << escape_json_string(state.status);
    ss << ",\"rate_hz\":" << std::setprecision(1) << state.rate_hz;

    if (!state.msg_type.empty()) {
      ss << ",\"msg_type\":" << escape_json_string(state.msg_type);
    } else {
      ss << ",\"msg_type\":null";
    }

    if (state.ever_received) {
      double age = std::chrono::duration<double>(now - state.last_received).count();
      ss << ",\"last_received_ago_s\":" << std::setprecision(2) << age;
    } else {
      ss << ",\"last_received_ago_s\":null";
    }

    ss << "}";
  }

  ss << "}}";
  return ss.str();
}

void TopicHealthchecker::run_http_server()
{
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create HTTP socket: %s", std::strerror(errno));
    return;
  }

  int opt = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(http_port_));

  if (::bind(server_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to bind HTTP port %d: %s", http_port_, std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  if (::listen(server_fd_, 8) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to listen on HTTP port: %s", std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Health HTTP server listening on port %d", http_port_);

  while (running_.load()) {
    int client_fd = ::accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (running_.load()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000, "HTTP accept failed: %s", std::strerror(errno));
      }
      continue;
    }

    // Read the request (we only need enough to see GET /health)
    char buf[512];
    ssize_t n = ::recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
      ::close(client_fd);
      continue;
    }
    buf[n] = '\0';

    std::string body;
    std::string status_line;

    // Check if it's GET /health
    if (std::strncmp(buf, "GET /health", 11) == 0) {
      body = build_health_json();
      status_line = "HTTP/1.1 200 OK\r\n";
    } else if (std::strncmp(buf, "GET / ", 6) == 0) {
      body = "{\"status\":\"ok\"}";
      status_line = "HTTP/1.1 200 OK\r\n";
    } else {
      body = "{\"error\":\"not found\"}";
      status_line = "HTTP/1.1 404 Not Found\r\n";
    }

    std::string response = status_line +
                           "Content-Type: application/json\r\n"
                           "Access-Control-Allow-Origin: *\r\n"
                           "Connection: close\r\n"
                           "Content-Length: " +
                           std::to_string(body.size()) +
                           "\r\n"
                           "\r\n" +
                           body;

    ::send(client_fd, response.c_str(), response.size(), MSG_NOSIGNAL);
    ::close(client_fd);
  }
}

}  // namespace topic_healthchecker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<topic_healthchecker::TopicHealthchecker>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
