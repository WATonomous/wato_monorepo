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

#include "carla_lifecycle/lifecycle_manager_node.hpp"

#include <chrono>
#include <algorithm>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using std::chrono_literals::operator""s;

namespace carla_lifecycle
{

LifecycleManagerNode::LifecycleManagerNode(const rclcpp::NodeOptions & options)
: Node("carla_lifecycle_manager", options)
{
  // Parameters
  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.description = "Automatically start managed nodes on startup";
  this->declare_parameter("autostart", true, desc);

  desc.description = "Name of the scenario server node to coordinate with";
  this->declare_parameter("scenario_server_name", "scenario_server", desc);

  desc.description = "List of lifecycle node names to manage";
  this->declare_parameter("node_names", std::vector<std::string>{}, desc);
  this->declare_parameter("optional_node_names", std::vector<std::string>{}, desc);

  desc.description = "Timeout for lifecycle service calls in seconds";
  this->declare_parameter("service_timeout", 10.0, desc);

  desc.description = "Retry interval when waiting for nodes in seconds";
  this->declare_parameter("startup_retry_interval", 5.0, desc);

  autostart_ = this->get_parameter("autostart").as_bool();
  scenario_server_name_ = this->get_parameter("scenario_server_name").as_string();
  node_names_ = this->get_parameter("node_names").as_string_array();
  optional_node_names_ = this->get_parameter("optional_node_names").as_string_array();
  service_timeout_ = this->get_parameter("service_timeout").as_double();
  startup_retry_interval_ = this->get_parameter("startup_retry_interval").as_double();

  RCLCPP_INFO(this->get_logger(), "Lifecycle manager initialized");
  RCLCPP_INFO(this->get_logger(), "  scenario_server: %s", scenario_server_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  managed nodes: %zu nodes", node_names_.size());

  // Create callback group for service clients
  service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize all service clients upfront
  auto init_clients = [this](const std::string & node_name) {
    std::string abs_name = toAbsoluteName(node_name);
    NodeClients nc;
    nc.change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(
      abs_name + "/change_state", rmw_qos_profile_services_default, service_cb_group_);
    nc.get_state = this->create_client<lifecycle_msgs::srv::GetState>(
      abs_name + "/get_state", rmw_qos_profile_services_default, service_cb_group_);
    clients_[node_name] = nc;
  };

  // Create clients for scenario server
  init_clients(scenario_server_name_);

  // Create clients for all managed nodes
  for (const auto & node_name : node_names_) {
    init_clients(node_name);
  }

  // Subscribe to scenario status
  scenario_sub_ = this->create_subscription<carla_msgs::msg::ScenarioStatus>(
    toAbsoluteName(scenario_server_name_) + "/scenario_status",
    10,
    std::bind(&LifecycleManagerNode::scenarioStatusCallback, this, std::placeholders::_1));

  // Service for scenario_server to request node cleanup before switching
  // Uses namespace-relative path so scenario_server can find it without knowing node name
  // Startup waits for scenario_server activation. Its callback must be able to
  // request bridge bringup while that wait is in progress.
  coordination_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  prepare_switch_service_ = this->create_service<std_srvs::srv::Trigger>(
    "prepare_for_scenario_switch",
    std::bind(&LifecycleManagerNode::prepareForSwitchCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, coordination_cb_group_);
  finish_switch_service_ = this->create_service<std_srvs::srv::Trigger>(
    "finish_scenario_switch",
    std::bind(&LifecycleManagerNode::finishSwitchCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, coordination_cb_group_);

  // Autostart timer - retries until scenario_server connects to CARLA
  if (autostart_) {
    auto interval = std::chrono::duration<double>(startup_retry_interval_);
    startup_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
      std::bind(&LifecycleManagerNode::startupTimerCallback, this));
    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for %s to connect to CARLA server (retry interval: %.1fs)...",
      scenario_server_name_.c_str(),
      startup_retry_interval_);
  }
}

void LifecycleManagerNode::startupTimerCallback()
{
  if (startup_complete_) {
    startup_timer_->cancel();
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Attempting to bring up %s (connecting to CARLA server)...", scenario_server_name_.c_str());

  if (bringUpNode(scenario_server_name_)) {
    RCLCPP_INFO(
      this->get_logger(), "%s successfully connected to CARLA server and is now active", scenario_server_name_.c_str());
    startup_complete_ = true;
    startup_timer_->cancel();
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to bring up %s (CARLA server may not be available), retrying in %.1fs...",
      scenario_server_name_.c_str(),
      startup_retry_interval_);
  }
}

void LifecycleManagerNode::scenarioStatusCallback(const carla_msgs::msg::ScenarioStatus::SharedPtr msg)
{
  // The server explicitly awaits finish_scenario_switch, including same-name
  // reloads. Status is observational and must not launch a competing transition.
  current_scenario_ = msg->scenario_name;
  last_scenario_state_ = msg->state;
}

void LifecycleManagerNode::prepareForSwitchCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Preparing for scenario switch: cleaning up managed nodes...");
  response->success = cleanupAllNodes();
  response->message = response->success ? "Managed nodes cleaned up" : "Bridge cleanup failed; inspect lifecycle logs";
  RCLCPP_INFO(this->get_logger(), "Managed nodes cleaned up, ready for scenario switch");
}

void LifecycleManagerNode::finishSwitchCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = bringUpAllNodes();
  response->message = response->success ? "Bridge nodes ready" : "Bridge bringup failed; inspect lifecycle logs";
}

bool LifecycleManagerNode::bringUpAllNodes()
{
  using Transition = lifecycle_msgs::msg::Transition;
  using State = lifecycle_msgs::msg::State;

  return executeTransitionSteps({
    {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_UNCONFIGURED, "configure"},
    {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_INACTIVE, "activate"},
  });
}

bool LifecycleManagerNode::cleanupAllNodes()
{
  using Transition = lifecycle_msgs::msg::Transition;
  using State = lifecycle_msgs::msg::State;

  return executeTransitionSteps({
    {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_ACTIVE, "deactivate"},
    {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_INACTIVE, "cleanup"},
  });
}

bool LifecycleManagerNode::executeTransitionSteps(const std::vector<TransitionStep> & steps)
{
  bool success = true;
  auto timeout = std::chrono::duration<double>(service_timeout_);

  for (const auto & step : steps) {
    using Future = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;
    std::vector<std::pair<std::string, Future>> futures;

    for (const auto & node_name : node_names_) {
      auto & client = clients_[node_name].change_state;
      if (!client->service_is_ready()) {
        if (std::find(optional_node_names_.begin(), optional_node_names_.end(), node_name) == optional_node_names_.end()) {
          RCLCPP_ERROR(this->get_logger(), "Required lifecycle service unavailable: %s", node_name.c_str());
          success = false;
        }
        continue;
      }
      int state = getNodeState(node_name);
      if (state < 0) success = false;
      if (state == step.required_state) {
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = step.transition_id;
        futures.emplace_back(node_name, client->async_send_request(req));
      }
    }

    for (auto & [node_name, future] : futures) {
      if (future.wait_for(timeout) == std::future_status::ready) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "%s: %s succeeded", node_name.c_str(), step.name);
        } else {
          success = false;
          RCLCPP_ERROR(this->get_logger(), "%s: %s failed", node_name.c_str(), step.name);
        }
      } else {
        success = false;
        RCLCPP_ERROR(this->get_logger(), "%s: %s timed out", node_name.c_str(), step.name);
      }
    }
  }
  const int expected = steps.back().transition_id == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE ?
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE : lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  for (const auto & node_name : node_names_) {
    if (!clients_[node_name].get_state->service_is_ready() &&
      std::find(optional_node_names_.begin(), optional_node_names_.end(), node_name) != optional_node_names_.end()) continue;
    if (getNodeState(node_name) != expected) success = false;
  }
  return success;
}

bool LifecycleManagerNode::bringUpNode(const std::string & node_name)
{
  auto it = clients_.find(node_name);
  if (it == clients_.end()) {
    RCLCPP_ERROR(this->get_logger(), "No clients for %s", node_name.c_str());
    return false;
  }

  auto & client = it->second.change_state;

  if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_))) {
    RCLCPP_WARN(this->get_logger(), "%s service not available", node_name.c_str());
    return false;
  }

  int state = getNodeState(node_name);
  if (state < 0) {
    RCLCPP_WARN(this->get_logger(), "Could not get state for %s", node_name.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Bringing up %s (state: %s)", node_name.c_str(), stateName(state).c_str());

  // Configure if unconfigured
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    if (!changeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return false;
    }
    state = getNodeState(node_name);
  }

  // Activate if inactive
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!changeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return false;
    }
  }

  return getNodeState(node_name) == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool LifecycleManagerNode::changeState(const std::string & node_name, uint8_t transition_id)
{
  auto & client = clients_[node_name].change_state;

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  auto future = client->async_send_request(request);
  // Scenario activation includes world generation and nested bridge bringup.
  auto timeout = std::chrono::duration<double>(
    node_name == scenario_server_name_ ? std::max(service_timeout_, 150.0) : service_timeout_);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "%s transition %d timed out", node_name.c_str(), transition_id);
    return false;
  }

  if (!future.get()->success) {
    RCLCPP_ERROR(this->get_logger(), "%s transition %d failed", node_name.c_str(), transition_id);
    return false;
  }

  return true;
}

int LifecycleManagerNode::getNodeState(const std::string & node_name)
{
  auto & client = clients_[node_name].get_state;
  auto timeout = std::chrono::duration<double>(service_timeout_);

  if (!client->wait_for_service(timeout)) {
    return -1;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    return -1;
  }

  return future.get()->current_state.id;
}

std::string LifecycleManagerNode::stateName(int state_id)
{
  switch (state_id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
      return "unknown";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return "unconfigured";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return "inactive";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return "active";
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      return "finalized";
    default:
      return std::to_string(state_id);
  }
}

std::string LifecycleManagerNode::toAbsoluteName(const std::string & name)
{
  if (name.empty()) {
    return "/";
  }
  if (name[0] == '/') {
    return name;
  }
  return "/" + name;
}

}  // namespace carla_lifecycle
