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

#include <deep_core/deep_node_base.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wato
{
namespace test
{

class DeepTestNode : public DeepNodeBase
{
public:
  explicit DeepTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : DeepNodeBase("wato_test_node", options)
  {}

  using DeepNodeBase::get_backend_name;
  using DeepNodeBase::get_current_allocator;
  using DeepNodeBase::is_model_loaded;
  using DeepNodeBase::is_plugin_loaded;
  using DeepNodeBase::load_model;
  using DeepNodeBase::load_plugin;
  using DeepNodeBase::run_inference;

protected:
  CallbackReturn on_configure_impl(const rclcpp_lifecycle::State & /*state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl(const rclcpp_lifecycle::State & /*state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl(const rclcpp_lifecycle::State & /*state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl(const rclcpp_lifecycle::State & /*state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown_impl(const rclcpp_lifecycle::State & /*state*/) override
  {
    return CallbackReturn::SUCCESS;
  }
};

}  // namespace test
}  // namespace wato
