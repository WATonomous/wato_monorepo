#include "eidos/plugins/motion_models/holonomic_motion_model.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/slam/BetweenFactor.h>

namespace eidos {

void HolonomicMotionModel::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;
  node_->declare_parameter(prefix + ".process_noise_diagonal",
      std::vector<double>{0.01, 0.01, 0.01, 0.1, 0.1, 0.1});
  node_->get_parameter(prefix + ".process_noise_diagonal", process_noise_diagonal_);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void HolonomicMotionModel::activate() {
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void HolonomicMotionModel::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void HolonomicMotionModel::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
}

void HolonomicMotionModel::generateMotionModel(
    gtsam::Key key_begin, double t_begin,
    gtsam::Key key_end, double t_end,
    gtsam::NonlinearFactorGraph& factors,
    gtsam::Values& /*values*/) {
  double dt = std::abs(t_end - t_begin);
  if (dt < 1e-9) dt = 1e-3;  // avoid zero noise

  // Scale noise by dt
  gtsam::Vector6 noise_vec;
  for (int i = 0; i < 6; i++) {
    noise_vec(i) = process_noise_diagonal_[i] * dt;
  }
  auto noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);

  factors.add(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_begin, key_end, gtsam::Pose3::Identity(), noise));
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::HolonomicMotionModel, eidos::MotionModelPlugin)
