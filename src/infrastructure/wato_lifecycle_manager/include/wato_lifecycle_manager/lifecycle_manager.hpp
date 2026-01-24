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

#ifndef WATO_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define WATO_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// Forward declaration for PIMPL - avoids exposing bondcpp in public header
namespace bond
{
class Bond;
}  // namespace bond

namespace wato_lifecycle_manager
{

/**
 * @brief Manages lifecycle transitions for a set of ROS2 lifecycle nodes.
 *
 * Responsibilities:
 * - Orchestrates startup/shutdown of multiple lifecycle nodes in order
 * - Uses bond connections to detect node crashes
 * - Provides services for manual control (startup, shutdown, reset)
 *
 * Managed nodes must:
 * 1. Be rclcpp_lifecycle::LifecycleNode instances
 * 2. Create a bond on activation: bond::Bond("/<node>/bond", ...)
 */
class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LifecycleManager() override;

protected:
  /**
   * @brief Information about a managed lifecycle node.
   */
  struct ManagedNode
  {
    std::string name;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
    std::unique_ptr<bond::Bond> bond;
  };

  /**
   * @brief Startup all managed nodes (configure -> activate in order).
   * @return true if all nodes reached active state
   */
  bool startup();

  /**
   * @brief Shutdown all managed nodes (deactivate -> cleanup in reverse order).
   * @return true if all nodes reached unconfigured state
   */
  bool shutdown();

  /**
   * @brief Reset all managed nodes (shutdown then startup).
   * @return true if reset succeeded
   */
  bool reset();

  /**
   * @brief Check if all managed nodes are currently active.
   * @return true if all nodes are in active state
   */
  bool isActive();

  /**
   * @brief Pause all managed nodes (deactivate only, stay configured).
   * @return true if all nodes reached inactive state
   */
  bool pause();

  /**
   * @brief Resume all managed nodes (activate from inactive).
   * @return true if all nodes reached active state
   */
  bool resume();

private:
  // Initialization
  void createManagedNodes();
  void createBond(ManagedNode & node);
  void destroyBond(ManagedNode & node);

  // State transitions
  bool changeState(ManagedNode & node, uint8_t transition_id, const std::chrono::seconds & timeout);

  bool waitForService(rclcpp::ClientBase::SharedPtr client, const std::chrono::seconds & timeout);

  uint8_t getNodeState(ManagedNode & node);

  // Bond callbacks
  void onBondBroken(const std::string & node_name);

  // Service callbacks
  void handleStartup(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleShutdown(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleIsActive(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handlePause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleResume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Managed nodes
  std::vector<ManagedNode> nodes_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startup_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  // Parameters
  std::vector<std::string> node_names_;
  bool autostart_;
  double transition_timeout_s_;
  double bond_timeout_s_;
  bool bond_enabled_;

  // State
  bool system_active_;
  std::mutex state_mutex_;

  // Callback group for service calls (to avoid deadlock)
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Autostart timer (one-shot)
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace wato_lifecycle_manager

#endif  // WATO_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
