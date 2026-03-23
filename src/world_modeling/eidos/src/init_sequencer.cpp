#include "eidos/core/init_sequencer.hpp"

namespace eidos {

void InitSequencer::configure(
    rclcpp::Logger logger,
    std::atomic<SlamState>* state,
    const PluginRegistry* registry,
    const MapManager* map_manager,
    double relocalization_timeout,
    TrackingCallback on_tracking) {
  logger_ = logger;
  state_ = state;
  registry_ = registry;
  map_manager_ = map_manager;
  relocalization_timeout_ = relocalization_timeout;
  on_tracking_ = std::move(on_tracking);
}

void InitSequencer::reset() {
  state_->store(SlamState::WARMING_UP, std::memory_order_release);
  relocalization_start_time_ = 0.0;
}

bool InitSequencer::step(double timestamp) {
  auto current = state_->load(std::memory_order_acquire);

  switch (current) {
    case SlamState::INITIALIZING:
      // No-op — transition to WARMING_UP happens in reset() / on_activate
      break;
    case SlamState::WARMING_UP:
      handleWarmingUp(timestamp);
      break;
    case SlamState::RELOCALIZING:
      handleRelocalizing(timestamp);
      break;
    case SlamState::TRACKING:
      return true;  // caller takes over
  }

  return state_->load(std::memory_order_acquire) == SlamState::TRACKING;
}

void InitSequencer::handleWarmingUp(double timestamp) {
  // Check if motion model is ready
  if (registry_->motion_model && !registry_->motion_model->isReady()) {
    return;
  }

  // If prior map loaded, enter relocalization
  if (map_manager_->hasPriorMap()) {
    state_->store(SlamState::RELOCALIZING, std::memory_order_release);
    relocalization_start_time_ = timestamp;
    RCLCPP_INFO(logger_, "\033[35m[RELOCALIZING]\033[0m Prior map loaded, entering relocalization");
    return;
  }

  // No prior map — begin tracking from origin
  on_tracking_(gtsam::Pose3::Identity());
}

void InitSequencer::handleRelocalizing(double timestamp) {
  // Keep factor plugins warm
  for (auto& plugin : registry_->factor_plugins) {
    plugin->produceFactor(0, timestamp);
  }

  // Try each relocalization plugin
  for (auto& plugin : registry_->reloc_plugins) {
    auto result = plugin->tryRelocalize(timestamp);
    if (result.has_value()) {
      RCLCPP_INFO(logger_,
                  "\033[35m[RELOCALIZING]\033[0m Succeeded (fitness: %.3f, keyframe: %d)",
                  result->fitness_score, result->matched_keyframe_index);
      on_tracking_(result->pose);
      return;
    }
  }

  // Timeout fallback
  if (timestamp - relocalization_start_time_ > relocalization_timeout_) {
    RCLCPP_WARN(logger_,
                "\033[35m[RELOCALIZING]\033[0m Timed out after %.1f seconds, starting from origin",
                timestamp - relocalization_start_time_);
    on_tracking_(gtsam::Pose3::Identity());
  }
}

}  // namespace eidos
