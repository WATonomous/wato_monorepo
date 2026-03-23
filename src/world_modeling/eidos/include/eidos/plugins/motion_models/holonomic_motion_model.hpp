#pragma once

#include "eidos/plugins/base_motion_model_plugin.hpp"

namespace eidos {

/**
 * @brief No-op motion model. Produces no odom — for systems without IMU.
 */
class HolonomicMotionModel : public MotionModelPlugin {
public:
  HolonomicMotionModel() = default;
  ~HolonomicMotionModel() override = default;

  void onInitialize() override {
    RCLCPP_INFO(node_->get_logger(), "[%s] initialized (no-op)", name_.c_str());
  }
  void activate() override {
    RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
  }
  void deactivate() override {
    RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
  }
};

}  // namespace eidos
