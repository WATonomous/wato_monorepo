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

#include "eidos/plugins/factors/motion_model_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

void MotionModelFactor::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".predict_service", std::string("predict_relative_transform"));
  node_->declare_parameter(prefix + ".noise_cov", std::vector<double>{1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4});

  std::string service_name;
  node_->get_parameter(prefix + ".predict_service", service_name);
  node_->get_parameter(prefix + ".noise_cov", noise_cov_);

  predict_client_ = node_->create_client<eidos_msgs::srv::PredictRelativeTransform>(service_name);

  RCLCPP_INFO(
    node_->get_logger(), "[%s] initialized (motion model factor, service=%s)", name_.c_str(), service_name.c_str());
}

void MotionModelFactor::activate()
{
  active_ = true;
  has_last_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void MotionModelFactor::deactivate()
{
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

StampedFactorResult MotionModelFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_) return result;

  // Get the owner of this state from the most recent produceFactor call
  // (passed through the factor_owners tracking in handleTracking)
  // For now, we always produce a between-factor for consecutive states
  // regardless of owner — the service handles the prediction.

  if (!has_last_) {
    last_key_ = key;
    last_timestamp_ = timestamp;
    has_last_ = true;
    return result;
  }

  // Skip if timestamps are too close
  double dt = timestamp - last_timestamp_;
  if (dt < 0.01) {
    last_key_ = key;
    last_timestamp_ = timestamp;
    return result;
  }

  // Call eidos_transform's predict service
  if (!predict_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "[%s] predict service not available", name_.c_str());
    last_key_ = key;
    last_timestamp_ = timestamp;
    return result;
  }

  auto request = std::make_shared<eidos_msgs::srv::PredictRelativeTransform::Request>();
  request->timestamp_from = last_timestamp_;
  request->timestamp_to = timestamp;

  auto future = predict_client_->async_send_request(request);

  // Wait briefly for the response (service is on the same machine, <1ms expected)
  if (future.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "[%s] predict service timeout", name_.c_str());
    last_key_ = key;
    last_timestamp_ = timestamp;
    return result;
  }

  auto response = future.get();
  if (!response->success) {
    last_key_ = key;
    last_timestamp_ = timestamp;
    return result;
  }

  // Convert response pose to gtsam::Pose3
  auto & p = response->relative_pose;
  gtsam::Rot3 rot = gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  gtsam::Point3 trans(p.position.x, p.position.y, p.position.z);
  gtsam::Pose3 relative_pose(rot, trans);

  // Build noise model
  gtsam::Vector6 noise_vec;
  for (int i = 0; i < 6 && i < static_cast<int>(noise_cov_.size()); i++) {
    noise_vec(i) = noise_cov_[i];
  }
  // Scale noise by dt — longer predictions are less certain
  noise_vec *= std::max(dt, 0.1);
  auto noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);

  result.factors.push_back(
    gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_key_, key, relative_pose, noise));

  gtsam::Symbol prev_sym(last_key_);
  gtsam::Symbol curr_sym(key);
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[34m[%s] MotionModelBetween (%c,%lu)->(%c,%lu) dt=%.3f\033[0m",
    name_.c_str(),
    prev_sym.chr(),
    prev_sym.index(),
    curr_sym.chr(),
    curr_sym.index(),
    dt);

  last_key_ = key;
  last_timestamp_ = timestamp;
  return result;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::MotionModelFactor, eidos::FactorPlugin)
