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

#include "eidos/plugins/motion_models/ackermann_motion_model.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace eidos
{

void AckermannMotionModel::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".twist_topic", std::string("ackermann_twist"));
  node_->declare_parameter(prefix + ".wheelbase", wheelbase_);

  std::string twist_topic;
  node_->get_parameter(prefix + ".twist_topic", twist_topic);
  node_->get_parameter(prefix + ".wheelbase", wheelbase_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    twist_topic, rclcpp::SensorDataQoS(),
    std::bind(&AckermannMotionModel::twistCallback, this, std::placeholders::_1), sub_opts);

  RCLCPP_INFO(
    node_->get_logger(), "[%s] initialized (ackermann, wheelbase=%.2f, topic=%s)",
    name_.c_str(), wheelbase_, twist_topic.c_str());
}

void AckermannMotionModel::activate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void AckermannMotionModel::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

gtsam::Pose3 AckermannMotionModel::predict(const gtsam::Pose3 & current, double dt)
{
  if (dt <= 0.0) return current;

  double v, steer;
  {
    std::lock_guard lock(data_mtx_);
    if (!has_data_) return current;
    v = speed_;
    steer = steering_angle_;
  }

  // Ackermann kinematics in body frame
  double dx = v * dt;
  double dyaw = (wheelbase_ > 0.0) ? v * std::tan(steer) / wheelbase_ * dt : 0.0;

  gtsam::Pose3 delta(gtsam::Rot3::Rz(dyaw), gtsam::Point3(dx, 0, 0));
  return current.compose(delta);
}

bool AckermannMotionModel::isReady() const
{
  return true;
}

void AckermannMotionModel::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard lock(data_mtx_);
  speed_ = msg->twist.linear.x;
  steering_angle_ = msg->twist.angular.z;
  has_data_ = true;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::AckermannMotionModel, eidos::MotionModelPlugin)
