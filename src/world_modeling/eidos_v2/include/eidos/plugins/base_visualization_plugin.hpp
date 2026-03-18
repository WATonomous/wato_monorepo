#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <gtsam/nonlinear/Values.h>

#include "eidos/utils/atomic_slot.hpp"

namespace eidos {

class MapManager;

/**
 * @brief Base class for read-only visualization plugins.
 *
 * Each visualization plugin runs on its own timer and callback group,
 * reading optimized values from an AtomicSlot on the Estimator (lock-free).
 * Plugins never block each other — each ticks independently.
 *
 * Subclasses implement render() to publish RViz messages.
 * The base class handles the timer lifecycle.
 */
class VisualizationPlugin {
public:
  virtual ~VisualizationPlugin() = default;

  const std::string& getName() const { return name_; }

  void initialize(
      const std::string& name,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      tf2_ros::Buffer* tf,
      MapManager* map_manager,
      const AtomicSlot<gtsam::Values>* values_slot,
      double rate_hz) {
    name_ = name;
    node_ = node;
    tf_ = tf;
    map_manager_ = map_manager;
    values_slot_ = values_slot;

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VisualizationPlugin::tick, this),
        callback_group_);
    timer_->cancel();  // starts inactive, activate() enables it

    onInitialize();
  }

  virtual void onInitialize() = 0;

  void activate() {
    timer_->reset();
    onActivate();
  }

  void deactivate() {
    timer_->cancel();
    onDeactivate();
  }

protected:
  /// Subclass lifecycle hooks.
  virtual void onActivate() {}
  virtual void onDeactivate() {}

  /// Subclass implements this to publish visualization messages.
  /// Called at the configured rate with the latest optimized values.
  /// Only called when new values are available.
  virtual void render(const gtsam::Values& optimized_values) = 0;

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer* tf_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  MapManager* map_manager_ = nullptr;

private:
  void tick() {
    auto values = values_slot_->load();
    if (!values) return;
    render(*values);
  }

  const AtomicSlot<gtsam::Values>* values_slot_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace eidos
