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

#include <array>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wato_fault_msgs/msg/fault.hpp>

#include "fault_collector/fault_collector_core.hpp"

namespace fault_collector
{

/**
 * @class FaultCollectorNode
 * @brief ROS 2 lifecycle node that aggregates /faults reports into diagnostics and a
 * two-tier fault response: a one-shot vehicle disarm and a continuous emergency brake.
 *
 * Subscribes to fault reports from any node in the monorepo, feeds them into a
 * FaultCollectorCore, and periodically publishes a diagnostic_msgs/DiagnosticArray (for
 * Foxglove's Diagnostics panel). It also drives two independent response tiers:
 *
 *   - Tier A (disarm): on the rising edge into "disarm requested" (per
 *     FaultCollectorCore::consume_disarm_edge()), calls the real vehicle authority's
 *     `std_srvs/SetBool` arm service (default `/oscc_interfacing/arm`) with `data: false`,
 *     exactly once. This node never modifies `oscc_interfacing`, never subscribes to its
 *     internal state, and never places any lockout on re-arming -- see the core's
 *     class-level comment for why the call is one-shot rather than a repeating/reactive one.
 *     The operator's arm button always works, including immediately after a disarm.
 *   - Tier B (brake): publishes a latched `emergency_stop` Bool that `ackermann_mux`
 *     treats as a hard override, continuously reflecting FaultCollectorCore::should_brake().
 *     If the Tier A disarm attempt fails (service unreachable or returns success=false),
 *     this node forces `emergency_stop` true regardless of `brake_level` as a fallback --
 *     if the vehicle cannot be disarmed, it must at least be braked.
 */
class FaultCollectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructs the FaultCollectorNode and declares all parameters.
   * @param options Node options for configuring the ROS 2 node.
   */
  explicit FaultCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configures the node: reads and validates parameters, builds the core, and
   * creates the subscription, publishers, service, and arm service client.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates the node, starting the periodic diagnostics/response timer.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates the node, stopping the periodic timer.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleans up the node, releasing all resources.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shuts down the node.
   * @param state The current state of the node.
   * @return CallbackReturn Success or Failure.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Callback for incoming fault reports on `faults_topic_`.
   * @param msg The received Fault message.
   */
  void fault_callback(const wato_fault_msgs::msg::Fault::ConstSharedPtr msg);

  /**
   * @brief Timer callback: expires stale faults, fires the one-shot disarm request on a
   * rising edge, publishes /diagnostics + emergency_stop + autonomy_disarm, and logs
   * response transitions.
   */
  void publish_diagnostics();

  /**
   * @brief Callback for `oscc_interfacing`'s `is_armed` signal — enforces the re-engage lockout.
   *
   * On a false->true transition (someone armed the vehicle), if FaultCollectorCore::disarm_required()
   * still holds, immediately re-issues the disarm request. This is what stops an operator from
   * arming back out of a persistent disarm-level fault, and it is enforced entirely here —
   * `oscc_interfacing` is never modified by this package.
   *
   * @param msg The received Bool; true = vehicle armed.
   */
  void is_armed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /**
   * @brief Sends a `SetBool(false)` disarm request to `arm_service_`.
   *
   * Shared by both trigger paths (the rising edge in publish_diagnostics(), and a re-arm
   * observed by is_armed_callback()). Rate-limited by `min_disarm_interval_s_` so an
   * arm/disarm fight cannot flood the service. If the service is not ready, immediately sets
   * `disarm_failed_` so the emergency brake fallback engages. Otherwise sends the request
   * asynchronously; the outcome is handled by handle_disarm_response().
   *
   * @param reason Human-readable cause, included in the log message.
   */
  void request_disarm(const char * reason);

  /**
   * @brief Handles the async response to a disarm request.
   * @param future Future resolving to the SetBool response.
   */
  void handle_disarm_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);

  /**
   * @brief Service callback for `~/clear_faults`. Drops all faults, releases both latches
   * and the disarm edge detector, and clears the disarm-failure fallback flag.
   * @param request Empty Trigger request.
   * @param response Populated with success=true and a summary message.
   */
  void handle_clear_faults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Builds a comma-separated "source: id-or-message" list of active faults at or
   * below the given severity level, for use in log messages.
   * @param level Severity threshold (matches disarm_level_ or brake_level_).
   * @return Human-readable list of offending faults.
   */
  std::string describe_faults_at_or_below(uint8_t level) const;

  // Params
  std::string faults_topic_;  ///< Topic to subscribe to for incoming Fault reports.
  std::string diagnostics_topic_;  ///< Topic to publish the DiagnosticArray on.
  std::string emergency_stop_topic_;  ///< Topic to publish the Tier B emergency_stop Bool on.
  std::string autonomy_disarm_topic_;  ///< Topic to publish the Tier A status Bool on (observability only).
  std::string arm_service_;  ///< Service name for oscc_interfacing's SetBool arm service.
  std::string is_armed_topic_;  ///< Topic carrying oscc_interfacing's is_armed signal.
  double min_disarm_interval_s_{};  ///< Minimum spacing between disarm service calls.
  double publish_rate_hz_{};  ///< Rate at which diagnostics/response signals are published.
  double fault_timeout_s_{};  ///< Faults not re-reported within this window expire.
  int64_t disarm_level_{};  ///< Tier A: faults at or below this level request a disarm.
  int64_t brake_level_{};  ///< Tier B: faults at or below this level request an emergency brake.
  bool latch_{};  ///< Whether a tripped tier latches until explicitly cleared.
  bool disarm_enabled_{};  ///< Whether Tier A actually calls the arm service (false = observe only).
  std::string diagnostic_name_prefix_;  ///< Prefix used for DiagnosticStatus names.
  std::array<uint8_t, 4> diagnostic_level_map_{2, 2, 1, 0};  ///< Fault level 1..4 -> DiagnosticStatus level.

  // Core — pure logic, no ROS dependencies.
  std::unique_ptr<FaultCollectorCore> core_;  ///< Fault table and two-tier response arbitration.

  // IO
  rclcpp::Subscription<wato_fault_msgs::msg::Fault>::SharedPtr fault_sub_;  ///< Incoming fault reports.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub_;  ///< oscc_interfacing arm-state feedback.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;  ///< /diagnostics output.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;  ///< Tier B latched brake output.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomy_disarm_pub_;  ///< Tier A status output (observability).
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;  ///< Client for oscc_interfacing's arm service.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_srv_;  ///< ~/clear_faults service.
  rclcpp::TimerBase::SharedPtr timer_;  ///< Periodic diagnostics/response timer.

  bool last_brake_state_{false};  ///< Previous tick's emergency_stop state, for transition logging.
  bool disarm_failed_{false};  ///< True if the last disarm attempt failed; forces emergency_stop true.
  bool is_armed_{false};  ///< Latest is_armed value from oscc_interfacing.
  bool has_is_armed_msg_{false};  ///< Whether an is_armed message has ever been received.
  rclcpp::Time last_disarm_call_time_{0, 0, RCL_ROS_TIME};  ///< When the last disarm request was sent.
};

}  // namespace fault_collector
