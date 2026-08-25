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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <roscco_msg/msg/roscco.hpp>

#include "oscc_mux/input_handle.hpp"

namespace oscc_mux
{

class OsccMuxNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit OsccMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void build_inputs_from_params();
  void mux_callback();

  /**
   * @brief Starts a handover ramp: captures last_published_ as the blend start and
   * resets handover_start_time_. Mirrors oscc_interfacing's begin_engage().
   */
  void begin_handover();

  /**
   * @brief Current blend authority in [0, 1] between handover_start_cmd_ and the
   * new winner's live command. Mirrors oscc_interfacing's engage_authority_scale().
   */
  float handover_authority_scale() const;

  /**
   * @brief Ends the in-progress handover once the ramp window elapses. Mirrors
   * oscc_interfacing's tick_engage().
   */
  void tick_handover();

  double safety_threshold_{};
  double publish_rate_hz_{};
  roscco_msg::msg::Roscco emergency_{};

  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr pub_out_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::shared_ptr<InputHandle>> inputs_;

  // Handover smoothing — blends from the previous winner's last published
  // command to the new winner's live command whenever the winning input changes.
  bool enable_handover_ramp_{true};
  double handover_ramp_ms_{600.0};

  std::shared_ptr<InputHandle> previous_winner_;
  bool in_handover_{false};
  roscco_msg::msg::Roscco handover_start_cmd_{};
  rclcpp::Time handover_start_time_;
  roscco_msg::msg::Roscco last_published_{};
};

}  // namespace oscc_mux
