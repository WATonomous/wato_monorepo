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

#include "wato_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace wato_lifecycle_manager
{

using std::chrono_literals::operator""s;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options)
, system_active_(false)
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("node_names", std::vector<std::string>{});
  this->declare_parameter<bool>("autostart", false);
  this->declare_parameter<double>("transition_timeout_s", 10.0);
  this->declare_parameter<double>("bond_timeout_s", 4.0);
  this->declare_parameter<bool>("bond_enabled", true);

  // Get parameters
  node_names_ = this->get_parameter("node_names").as_string_array();
  autostart_ = this->get_parameter("autostart").as_bool();
  transition_timeout_s_ = this->get_parameter("transition_timeout_s").as_double();
  bond_timeout_s_ = this->get_parameter("bond_timeout_s").as_double();
  bond_enabled_ = this->get_parameter("bond_enabled").as_bool();

  if (node_names_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No node_names provided, lifecycle manager has nothing to manage");
  }

  // Create callback group for async service calls
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create managed node clients
  createManagedNodes();

  // Create services
  startup_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/startup", std::bind(&LifecycleManager::handleStartup, this, std::placeholders::_1, std::placeholders::_2));

  shutdown_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/shutdown", std::bind(&LifecycleManager::handleShutdown, this, std::placeholders::_1, std::placeholders::_2));

  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&LifecycleManager::handleReset, this, std::placeholders::_1, std::placeholders::_2));

  is_active_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/is_active", std::bind(&LifecycleManager::handleIsActive, this, std::placeholders::_1, std::placeholders::_2));

  pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/pause", std::bind(&LifecycleManager::handlePause, this, std::placeholders::_1, std::placeholders::_2));

  resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/resume", std::bind(&LifecycleManager::handleResume, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Lifecycle manager initialized with %zu nodes", node_names_.size());

  // Autostart if configured
  if (autostart_ && !node_names_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Autostart enabled, starting managed nodes...");
    // Use a one-shot timer to allow node to fully initialize first
    autostart_timer_ = this->create_wall_timer(1s, [this]() {
      // Cancel timer first to make it one-shot
      autostart_timer_->cancel();
      if (startup()) {
        RCLCPP_INFO(this->get_logger(), "Autostart completed successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Autostart failed");
      }
    });
  }
}

LifecycleManager::~LifecycleManager()
{
  // Destroy all bonds on shutdown
  for (auto & node : nodes_) {
    destroyBond(node);
  }
}

void LifecycleManager::createManagedNodes()
{
  for (const auto & name : node_names_) {
    ManagedNode node;
    node.name = name;

    // Create lifecycle service clients
    node.change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + name + "/change_state", rmw_qos_profile_services_default, callback_group_);

    node.get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(
      "/" + name + "/get_state", rmw_qos_profile_services_default, callback_group_);

    nodes_.push_back(std::move(node));
    RCLCPP_DEBUG(this->get_logger(), "Created clients for managed node: %s", name.c_str());
  }
}

void LifecycleManager::createBond(ManagedNode & node)
{
  if (!bond_enabled_) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Creating bond for node: %s", node.name.c_str());

  node.bond = std::make_unique<bond::Bond>("/" + node.name + "/bond", node.name, shared_from_this());

  node.bond->setHeartbeatTimeout(bond_timeout_s_);
  node.bond->setHeartbeatPeriod(bond_timeout_s_ / 4.0);

  node.bond->setBrokenCallback(std::bind(&LifecycleManager::onBondBroken, this, node.name));

  node.bond->start();
}

void LifecycleManager::destroyBond(ManagedNode & node)
{
  if (node.bond) {
    node.bond->breakBond();
    node.bond.reset();
  }
}

void LifecycleManager::onBondBroken(const std::string & node_name)
{
  RCLCPP_ERROR(this->get_logger(), "Bond broken for node: %s - node may have crashed!", node_name.c_str());

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (system_active_) {
    RCLCPP_WARN(this->get_logger(), "System was active, initiating shutdown due to node failure");
    system_active_ = false;
    // Could trigger recovery here - for now just log
    // Consider: shutdown(), or attempt to restart the failed node
  }
}

bool LifecycleManager::waitForService(rclcpp::ClientBase::SharedPtr client, const std::chrono::seconds & timeout)
{
  auto start = std::chrono::steady_clock::now();
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      return false;
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > timeout) {
      return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Waiting for service: %s", client->get_service_name());
  }
  return true;
}

bool LifecycleManager::changeState(ManagedNode & node, uint8_t transition_id, const std::chrono::seconds & timeout)
{
  if (!waitForService(node.change_state_client, timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Service not available for node: %s", node.name.c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  auto future = node.change_state_client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state change on node: %s", node.name.c_str());
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change state for node: %s", node.name.c_str());
    return false;
  }

  return true;
}

uint8_t LifecycleManager::getNodeState(ManagedNode & node)
{
  auto timeout = std::chrono::seconds(static_cast<int>(transition_timeout_s_));

  if (!waitForService(node.get_state_client, timeout)) {
    RCLCPP_ERROR(this->get_logger(), "GetState service not available for node: %s", node.name.c_str());
    return State::PRIMARY_STATE_UNKNOWN;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = node.get_state_client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout getting state for node: %s", node.name.c_str());
    return State::PRIMARY_STATE_UNKNOWN;
  }

  return future.get()->current_state.id;
}

bool LifecycleManager::startup()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(this->get_logger(), "Starting up managed nodes...");

  auto timeout = std::chrono::seconds(static_cast<int>(transition_timeout_s_));

  // Configure and activate each node in order
  for (auto & node : nodes_) {
    RCLCPP_INFO(this->get_logger(), "Bringing up node: %s", node.name.c_str());

    // Get current state
    uint8_t state = getNodeState(node);

    // If unconfigured, configure first
    if (state == State::PRIMARY_STATE_UNCONFIGURED) {
      RCLCPP_INFO(this->get_logger(), "Configuring node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_CONFIGURE, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure node: %s", node.name.c_str());
        return false;
      }
      state = State::PRIMARY_STATE_INACTIVE;
    }

    // If inactive, activate
    if (state == State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(this->get_logger(), "Activating node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_ACTIVATE, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to activate node: %s", node.name.c_str());
        return false;
      }
    }

    // Create bond after activation
    createBond(node);

    RCLCPP_INFO(this->get_logger(), "Node %s is now active", node.name.c_str());
  }

  system_active_ = true;
  RCLCPP_INFO(this->get_logger(), "All managed nodes are now active");
  return true;
}

bool LifecycleManager::shutdown()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(this->get_logger(), "Shutting down managed nodes...");

  auto timeout = std::chrono::seconds(static_cast<int>(transition_timeout_s_));

  // Deactivate and cleanup each node in reverse order
  for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
    auto & node = *it;
    RCLCPP_INFO(this->get_logger(), "Shutting down node: %s", node.name.c_str());

    // Destroy bond first
    destroyBond(node);

    // Get current state
    uint8_t state = getNodeState(node);

    // If active, deactivate first
    if (state == State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(this->get_logger(), "Deactivating node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_DEACTIVATE, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deactivate node: %s", node.name.c_str());
        return false;
      }
      state = State::PRIMARY_STATE_INACTIVE;
    }

    // If inactive, cleanup
    if (state == State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(this->get_logger(), "Cleaning up node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_CLEANUP, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to cleanup node: %s", node.name.c_str());
        return false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Node %s is now unconfigured", node.name.c_str());
  }

  system_active_ = false;
  RCLCPP_INFO(this->get_logger(), "All managed nodes are now shut down");
  return true;
}

bool LifecycleManager::reset()
{
  RCLCPP_INFO(this->get_logger(), "Resetting managed nodes...");

  if (!shutdown()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to shutdown during reset");
    return false;
  }

  if (!startup()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to startup during reset");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Reset completed successfully");
  return true;
}

bool LifecycleManager::isActive()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (auto & node : nodes_) {
    uint8_t state = getNodeState(node);
    if (state != State::PRIMARY_STATE_ACTIVE) {
      return false;
    }
  }
  return true;
}

bool LifecycleManager::pause()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(this->get_logger(), "Pausing managed nodes...");

  auto timeout = std::chrono::seconds(static_cast<int>(transition_timeout_s_));

  // Deactivate each node in reverse order (but don't cleanup)
  for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
    auto & node = *it;

    // Destroy bond
    destroyBond(node);

    uint8_t state = getNodeState(node);

    if (state == State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(this->get_logger(), "Deactivating node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_DEACTIVATE, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deactivate node: %s", node.name.c_str());
        return false;
      }
    }
  }

  system_active_ = false;
  RCLCPP_INFO(this->get_logger(), "All managed nodes are now paused (inactive)");
  return true;
}

bool LifecycleManager::resume()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(this->get_logger(), "Resuming managed nodes...");

  auto timeout = std::chrono::seconds(static_cast<int>(transition_timeout_s_));

  // Activate each node in order
  for (auto & node : nodes_) {
    uint8_t state = getNodeState(node);

    if (state == State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(this->get_logger(), "Activating node: %s", node.name.c_str());
      if (!changeState(node, Transition::TRANSITION_ACTIVATE, timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to activate node: %s", node.name.c_str());
        return false;
      }
    }

    // Recreate bond
    createBond(node);
  }

  system_active_ = true;
  RCLCPP_INFO(this->get_logger(), "All managed nodes are now resumed (active)");
  return true;
}

// Service handlers

void LifecycleManager::handleStartup(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = startup();
  response->message = response->success ? "Startup successful" : "Startup failed";
}

void LifecycleManager::handleShutdown(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = shutdown();
  response->message = response->success ? "Shutdown successful" : "Shutdown failed";
}

void LifecycleManager::handleReset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = reset();
  response->message = response->success ? "Reset successful" : "Reset failed";
}

void LifecycleManager::handleIsActive(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = isActive();
  response->message = response->success ? "All nodes active" : "Not all nodes active";
}

void LifecycleManager::handlePause(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = pause();
  response->message = response->success ? "Pause successful" : "Pause failed";
}

void LifecycleManager::handleResume(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = resume();
  response->message = response->success ? "Resume successful" : "Resume failed";
}

}  // namespace wato_lifecycle_manager
