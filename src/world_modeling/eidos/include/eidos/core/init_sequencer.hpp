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

#include <gtsam/geometry/Pose3.h>

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eidos/core/map_manager.hpp"
#include "eidos/core/plugin_registry.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

/**
 * @brief Manages the INIT → WARMUP → RELOCALIZING → TRACKING state machine.
 *
 * Separated from EidosNode so the orchestrator doesn't mix state machine logic
 * with SLAM tracking logic. EidosNode calls step() each SLAM tick; once
 * InitSequencer reaches TRACKING, it calls the onTracking callback and
 * EidosNode takes over with handleSlamTracking / handleLocalizationTracking.
 *
 * Threading: only called from the SLAM loop thread.
 */
class InitSequencer
{
public:
  /// Callback invoked when the sequencer transitions to TRACKING.
  using TrackingCallback = std::function<void(const gtsam::Pose3 & initial_pose)>;

  void configure(
    rclcpp::Logger logger,
    std::atomic<SlamState> * state,
    const PluginRegistry * registry,
    const MapManager * map_manager,
    double relocalization_timeout,
    TrackingCallback on_tracking);

  /// Called each SLAM tick while state is not TRACKING.
  /// Returns true once TRACKING is reached (caller should switch to tracking handlers).
  bool step(double timestamp);

  /// Reset back to WARMING_UP (e.g. after beginTracking resets Estimator).
  void reset();

private:
  void handleWarmingUp(double timestamp);
  void handleRelocalizing(double timestamp);

  rclcpp::Logger logger_{rclcpp::get_logger("init_sequencer")};
  std::atomic<SlamState> * state_ = nullptr;
  const PluginRegistry * registry_ = nullptr;
  const MapManager * map_manager_ = nullptr;
  double relocalization_timeout_ = 30.0;
  double relocalization_start_time_ = 0.0;
  TrackingCallback on_tracking_;
};

}  // namespace eidos
