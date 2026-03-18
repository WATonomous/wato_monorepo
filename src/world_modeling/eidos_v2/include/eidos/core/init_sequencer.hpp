#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>

#include "eidos/utils/types.hpp"
#include "eidos/core/plugin_registry.hpp"
#include "eidos/core/map_manager.hpp"

namespace eidos {

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
class InitSequencer {
public:
  /// Callback invoked when the sequencer transitions to TRACKING.
  using TrackingCallback = std::function<void(const gtsam::Pose3& initial_pose)>;

  void configure(
      rclcpp::Logger logger,
      std::atomic<SlamState>* state,
      const PluginRegistry* registry,
      const MapManager* map_manager,
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
  std::atomic<SlamState>* state_ = nullptr;
  const PluginRegistry* registry_ = nullptr;
  const MapManager* map_manager_ = nullptr;
  double relocalization_timeout_ = 30.0;
  double relocalization_start_time_ = 0.0;
  TrackingCallback on_tracking_;
};

}  // namespace eidos
