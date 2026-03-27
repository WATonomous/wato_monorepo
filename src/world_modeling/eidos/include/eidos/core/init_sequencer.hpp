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

  /**
   * @brief Configure the sequencer with external dependencies.
   *
   * Must be called once (during EidosNode::on_configure) before step().
   * @param logger                 ROS logger for status messages.
   * @param state                  Pointer to the shared atomic SLAM state.
   * @param registry               Pointer to the plugin registry (read-only).
   * @param map_manager            Pointer to the map manager (read-only, for
   *                               checking prior map availability).
   * @param relocalization_timeout Maximum seconds to attempt relocalization
   *                               before falling back to starting from origin.
   * @param on_tracking            Callback invoked when transitioning to TRACKING.
   */
  void configure(
    rclcpp::Logger logger,
    std::atomic<SlamState> * state,
    const PluginRegistry * registry,
    const MapManager * map_manager,
    double relocalization_timeout,
    TrackingCallback on_tracking);

  /**
   * @brief Advance the state machine by one tick.
   *
   * Handles the INITIALIZING -> WARMING_UP -> RELOCALIZING -> TRACKING
   * progression. While in WARMING_UP, waits for all factor plugins to
   * report ready. In RELOCALIZING, polls relocalization plugins and
   * enforces the timeout.
   * @param timestamp Current wall-clock time in seconds.
   * @return True if the state is now TRACKING (caller should switch to
   *         tracking handlers), false otherwise.
   */
  bool step(double timestamp);

  /**
   * @brief Reset the state machine back to WARMING_UP.
   *
   * Typically called from on_activate() to restart the init sequence.
   */
  void reset();

private:
  /**
   * @brief Handle the WARMING_UP state: wait for all factor plugins to
   *        report ready, then transition to RELOCALIZING or TRACKING.
   * @param timestamp Current wall-clock time in seconds.
   */
  void handleWarmingUp(double timestamp);

  /**
   * @brief Handle the RELOCALIZING state: poll relocalization plugins for
   *        a match and enforce the relocalization timeout.
   * @param timestamp Current wall-clock time in seconds.
   */
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
