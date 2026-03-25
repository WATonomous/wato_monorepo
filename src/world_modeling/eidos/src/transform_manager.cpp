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

#include "eidos/core/transform_manager.hpp"

namespace eidos
{

void TransformManager::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const PluginRegistry * registry,
  const LockFreePose * estimator_pose,
  const std::string & map_frame,
  const std::string & odom_frame,
  const std::string & base_link_frame,
  const std::string & odom_source_name,
  const std::string & map_source_name,
  double rate_hz)
{
  node_ = node;
  registry_ = registry;
  estimator_pose_ = estimator_pose;
  map_frame_ = map_frame;
  odom_frame_ = odom_frame;
  base_link_frame_ = base_link_frame;
  odom_source_name_ = odom_source_name;
  map_source_name_ = map_source_name;

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Configure EKF if odom_source is set
  if (!odom_source_name_.empty()) {
    // Read fusion noise params — must be in config
    std::vector<double> process_noise, measurement_noise;
    node_->get_parameter("transforms.fusion.process_noise", process_noise);
    node_->get_parameter("transforms.fusion.measurement_noise", measurement_noise);

    Eigen::Matrix<double, 6, 1> Q_diag, R_diag;
    // Config is [roll,pitch,yaw,x,y,z], same order as GTSAM Pose3 Logmap
    for (int i = 0; i < 6 && i < static_cast<int>(process_noise.size()); i++) Q_diag(i) = process_noise[i];
    for (int i = 0; i < 6 && i < static_cast<int>(measurement_noise.size()); i++) R_diag(i) = measurement_noise[i];

    odom_ekf_.configure(Q_diag, R_diag);
  }

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TransformManager::tick, this),
    callback_group_);
  timer_->cancel();
}

void TransformManager::activate()
{
  odom_pub_->on_activate();
  timer_->reset();
}

void TransformManager::deactivate()
{
  timer_->cancel();
  odom_pub_->on_deactivate();
}

void TransformManager::tick()
{
  auto now = node_->now();

  // 1. Compute and broadcast odom → base_link
  auto odom_candidate = computeOdomToBase();
  // Guard: don't broadcast NaN poses
  auto t = odom_candidate.translation();
  auto q_check = odom_candidate.rotation().toQuaternion();
  bool sane = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
              std::isfinite(q_check.w()) && t.norm() < 1e6;
  if (sane) {
    cached_odom_to_base_ = odom_candidate;
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[TM] Bad odom->base (norm=%.2e), resetting EKF", t.norm());
    odom_ekf_.reset(gtsam::Pose3());
  }
  broadcastTf(now, odom_frame_, base_link_frame_, cached_odom_to_base_);

  // 1b. Publish unified fused odometry
  if (odom_pub_->is_activated()) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    auto q = cached_odom_to_base_.rotation().toQuaternion();
    odom_msg.pose.pose.position.x = cached_odom_to_base_.translation().x();
    odom_msg.pose.pose.position.y = cached_odom_to_base_.translation().y();
    odom_msg.pose.pose.position.z = cached_odom_to_base_.translation().z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg);
  }

  // 2. Read map-frame pose from configured source (lock-free)
  //    When map_source is empty (localization mode), map→odom is fixed from
  //    relocalization via setMapToOdom() and never recomputed.
  std::optional<gtsam::Pose3> map_pose;
  if (!map_source_name_.empty()) {
    if (map_source_name_ == "slam_core") {
      if (estimator_pose_) {
        map_pose = estimator_pose_->load();
      }
    } else {
      auto src = registry_->findFactor(map_source_name_);
      if (src) {
        map_pose = src->getMapPose();
      }
    }

    // 3. map→odom only changes when map_src changes (SLAM optimization).
    //    Between optimizations, map→odom is constant — odom→base carries all motion.
    if (map_pose) {
      if (!has_last_map_pose_ || !map_pose->equals(last_map_pose_, 1e-12)) {
        auto candidate = map_pose->compose(cached_odom_to_base_.inverse());
        auto t_m = candidate.translation();
        auto q_m = candidate.rotation().toQuaternion();
        if (std::isfinite(t_m.x()) && std::isfinite(t_m.y()) && std::isfinite(t_m.z()) &&
            std::isfinite(q_m.w()) && t_m.norm() < 1e6) {
          cached_map_to_odom_ = candidate;
        } else {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[TM] Bad map->odom (norm=%.2e), ignoring", t_m.norm());
        }
        last_map_pose_ = *map_pose;
        has_last_map_pose_ = true;
      }
    }
  }


  broadcastTf(now, map_frame_, odom_frame_, cached_map_to_odom_);

  // Write current map-frame robot pose (for viz, publishing, etc.)
  current_map_pose_.store(cached_map_to_odom_.compose(cached_odom_to_base_));
}

gtsam::Pose3 TransformManager::computeOdomToBase()
{
  // Compute dt since last tick
  auto now = std::chrono::steady_clock::now();
  double dt = 0.0;
  if (has_last_tick_time_) {
    dt = std::chrono::duration<double>(now - last_tick_time_).count();
  }
  last_tick_time_ = now;
  has_last_tick_time_ = true;

  // No odom_source — motion model IS the only source (no EKF)
  if (odom_source_name_.empty()) {
    if (registry_->motion_model) {
      return registry_->motion_model->predict(cached_odom_to_base_, dt);
    }
    return cached_odom_to_base_;
  }

  // ---- Fused odom: motion model prediction + odom_source correction ----

  // Read odom_source measurement
  auto src = registry_->findFactor(odom_source_name_);
  if (src) {
    auto odom_reading = src->getOdomPose();
    if (odom_reading) {
      if (!has_odom_source_reading_ || !odom_reading->equals(last_odom_source_pose_, 1e-12)) {
        last_odom_source_pose_ = *odom_reading;
        cached_odom_to_base_ = *odom_reading;  // snap to measurement
        has_odom_source_reading_ = true;

        // Feed corrected pose to motion model for velocity estimation
        if (registry_->motion_model) {
          double ts = node_->now().seconds();
          registry_->motion_model->onMeasurementUpdate(*odom_reading, ts);
        }
      }
    }
  }

  // Predict forward from current odom pose using motion model
  if (has_odom_source_reading_ && registry_->motion_model && dt > 0.0) {
    return registry_->motion_model->predict(cached_odom_to_base_, dt);
  }

  return has_odom_source_reading_ ? last_odom_source_pose_ : cached_odom_to_base_;
}

void TransformManager::broadcastTf(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & child_frame_id,
  const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;

  auto q = pose.rotation().toQuaternion();
  tf.transform.translation.x = pose.translation().x();
  tf.transform.translation.y = pose.translation().y();
  tf.transform.translation.z = pose.translation().z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  broadcaster_->sendTransform(tf);
}

}  // namespace eidos
