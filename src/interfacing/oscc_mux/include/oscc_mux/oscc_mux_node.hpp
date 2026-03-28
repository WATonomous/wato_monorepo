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

  double safety_threshold_{};
  double publish_rate_hz_{};
  roscco_msg::msg::Roscco emergency_{};

  rclcpp::Publisher<roscco_msg::msg::Roscco>::SharedPtr pub_out_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::shared_ptr<InputHandle>> inputs_;
};

}  // namespace oscc_mux
