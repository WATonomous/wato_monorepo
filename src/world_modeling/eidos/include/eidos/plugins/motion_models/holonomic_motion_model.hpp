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

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos
{

/**
 * @brief No-op motion model. Produces no odom — for systems without IMU.
 */
class HolonomicMotionModel : public MotionModelPlugin
{
public:
  HolonomicMotionModel() = default;
  ~HolonomicMotionModel() override = default;

  void onInitialize() override
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] initialized (no-op)", name_.c_str());
  }

  void activate() override
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
  }

  void deactivate() override
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
  }
};

}  // namespace eidos
