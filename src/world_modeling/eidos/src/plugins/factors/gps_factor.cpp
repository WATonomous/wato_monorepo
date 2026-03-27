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

#include "eidos/plugins/factors/gps_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

constexpr double kGpsTimeWindow = 0.2;
constexpr double kElevationLockedNoise = 1e6;
constexpr double kQuatLength2Min = 0.01;

// ==========================================================================
// Lifecycle
// ==========================================================================

void GpsFactor::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".gps_topic", "gps/fix");
  node_->declare_parameter(prefix + ".max_cov", 2.0);
  node_->declare_parameter(prefix + ".use_elevation", false);
  node_->declare_parameter(prefix + ".min_radius", 5.0);
  node_->declare_parameter(prefix + ".gps_cov", std::vector<double>{1.0, 1.0, 1.0});
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".pose_cov_threshold", 25.0);
  node_->declare_parameter(prefix + ".add_factors", true);

  std::string gps_topic, imu_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".max_cov", max_cov_);
  node_->get_parameter(prefix + ".use_elevation", use_elevation_);
  node_->get_parameter(prefix + ".min_radius", min_radius_);
  node_->get_parameter(prefix + ".gps_cov", gps_cov_);
  node_->get_parameter(prefix + ".pose_cov_threshold", pose_cov_threshold_);
  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter("frames.map", map_frame_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::SensorDataQoS(), std::bind(&GpsFactor::gpsCallback, this, std::placeholders::_1), sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&GpsFactor::imuCallback, this, std::placeholders::_1), sub_opts);

  // GPS factor owns its own static TF broadcaster for utm→map
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  utm_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/utm_pose", 10);

  // Register data formats with MapManager for persistence
  map_manager_->registerGlobalFormat("gps_factor/utm_to_map", "raw_double5");
  map_manager_->registerKeyframeFormat("gps_factor/position", "raw_double3");
  map_manager_->registerKeyframeFormat("gps_factor/utm_position", "raw_double4_eigen");

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (unary GPSFactor)", name_.c_str());
}

void GpsFactor::activate()
{
  active_ = true;
  utm_pub_->on_activate();

  // Check if a prior offset was loaded from a saved map
  auto offset_opt = map_manager_->retrieveGlobal<std::array<double, 5>>("gps_factor/utm_to_map");
  if (offset_opt.has_value()) {
    auto offset = *offset_opt;
    utm_to_map_offset_ = gtsam::Point3(offset[0], offset[1], offset[2]);
    int encoded = static_cast<int>(offset[3]);
    utm_zone_ = encoded / 10;
    utm_is_north_ = (encoded % 10) != 0;
    initial_yaw_ = offset[4];
    double cy = std::cos(-initial_yaw_);
    double sy = std::sin(-initial_yaw_);
    R_map_enu_ << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
    offset_initialized_ = true;
    broadcastUtmToMap();
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] loaded prior offset (zone %d%c, yaw=%.3f): [%.1f, %.1f, %.1f]",
      name_.c_str(),
      utm_zone_,
      utm_is_north_ ? 'N' : 'S',
      initial_yaw_,
      utm_to_map_offset_.x(),
      utm_to_map_offset_.y(),
      utm_to_map_offset_.z());
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsFactor::deactivate()
{
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// latchFactor — attach unary GPSFactor to an existing state
// ==========================================================================

StampedFactorResult GpsFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_) return result;

  std::lock_guard<std::mutex> lock(gps_lock_);
  if (gps_queue_.empty()) return result;

  // Time-match GPS fix to state timestamp
  while (!gps_queue_.empty() && rclcpp::Time(gps_queue_.front().header.stamp).seconds() < timestamp - kGpsTimeWindow) {
    gps_queue_.pop_front();
  }
  if (gps_queue_.empty()) return result;
  if (rclcpp::Time(gps_queue_.front().header.stamp).seconds() > timestamp + kGpsTimeWindow) return result;

  size_t best_idx = 0;
  double best_dt = std::abs(rclcpp::Time(gps_queue_[0].header.stamp).seconds() - timestamp);
  for (size_t i = 1; i < gps_queue_.size(); ++i) {
    double t = rclcpp::Time(gps_queue_[i].header.stamp).seconds();
    if (t > timestamp + kGpsTimeWindow) break;
    double dt = std::abs(t - timestamp);
    if (dt < best_dt) {
      best_dt = dt;
      best_idx = i;
    }
  }

  sensor_msgs::msg::NavSatFix this_fix = gps_queue_[best_idx];
  gps_queue_.erase(gps_queue_.begin(), gps_queue_.begin() + best_idx + 1);

  if (this_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return result;

  double sensor_cov_x = this_fix.position_covariance[0];
  double sensor_cov_y = this_fix.position_covariance[4];
  double sensor_cov_z = this_fix.position_covariance[8];
  if (sensor_cov_x > max_cov_ || sensor_cov_y > max_cov_) return result;

  UtmCoordinate utm = latLonToUtm(this_fix.latitude, this_fix.longitude, this_fix.altitude);
  Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);

  // Initialize offset on first accepted fix
  if (!offset_initialized_) {
    if (!has_imu_orientation_.load(std::memory_order_relaxed)) return result;

    utm_zone_ = utm.zone;
    utm_is_north_ = utm.is_north;
    {
      std::lock_guard<std::mutex> imu_lock(imu_orientation_lock_);
      initial_yaw_ = latest_imu_yaw_;
    }
    double cy = std::cos(-initial_yaw_);
    double sy = std::sin(-initial_yaw_);
    R_map_enu_ << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

    Eigen::Vector3d utm_rotated = utmToMap(utm_pos);
    auto current_pose = estimator_pose_->load().value_or(gtsam::Pose3::Identity());
    Eigen::Vector3d map_pos(
      current_pose.translation().x(), current_pose.translation().y(), current_pose.translation().z());
    utm_to_map_offset_ = gtsam::Point3(
      utm_rotated.x() - map_pos.x(),
      utm_rotated.y() - map_pos.y(),
      use_elevation_ ? (utm_rotated.z() - map_pos.z()) : 0.0);

    offset_initialized_ = true;
    broadcastUtmToMap();

    double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
    std::array<double, 5> global_offset = {
      utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z(), zone_encoded, initial_yaw_};
    map_manager_->storeGlobal("gps_factor/utm_to_map", global_offset);

    RCLCPP_INFO(
      node_->get_logger(),
      "\033[35m[%s] offset initialized: yaw=%.1f deg, offset=(%.1f, %.1f, %.1f)\033[0m",
      name_.c_str(),
      initial_yaw_ * 180.0 / M_PI,
      utm_to_map_offset_.x(),
      utm_to_map_offset_.y(),
      utm_to_map_offset_.z());
  }

  Eigen::Vector3d gps_rotated = utmToMap(utm_pos);
  double gps_x = gps_rotated.x() - utm_to_map_offset_.x();
  double gps_y = gps_rotated.y() - utm_to_map_offset_.y();
  double gps_z = use_elevation_ ? gps_rotated.z() - utm_to_map_offset_.z()
                                : estimator_pose_->load().value_or(gtsam::Pose3::Identity()).translation().z();

  // Distance gate
  float utm_x = static_cast<float>(utm_pos.x());
  float utm_y = static_cast<float>(utm_pos.y());
  if (has_last_gps_) {
    float dx = utm_x - last_gps_point_.x;
    float dy = utm_y - last_gps_point_.y;
    if (std::sqrt(dx * dx + dy * dy) < min_radius_) return result;
  }
  last_gps_point_.x = utm_x;
  last_gps_point_.y = utm_y;
  last_gps_point_.z = static_cast<float>(utm_pos.z());
  has_last_gps_ = true;

  double noise_x = std::max(sensor_cov_x, gps_cov_[0]);
  double noise_y = std::max(sensor_cov_y, gps_cov_[1]);
  double noise_z = use_elevation_ ? std::max(sensor_cov_z, gps_cov_[2]) : kElevationLockedNoise;
  auto gps_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << noise_x, noise_y, noise_z).finished());

  gtsam::Point3 gps_measurement(gps_x, gps_y, gps_z);
  if (add_factors_) {
    result.factors.push_back(gtsam::make_shared<gtsam::GPSFactor>(key, gps_measurement, gps_noise));
    result.correction = true;
  }

  map_manager_->store(key, "gps_factor/position", gps_measurement);

  double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
  Eigen::Vector4d utm_data(utm.easting, utm.northing, utm.altitude, zone_encoded);
  map_manager_->store(key, "gps_factor/utm_position", utm_data);

  gtsam::Symbol sym(key);
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[35m[%s] GPSFactor at (%c,%lu): map=(%.2f,%.2f,%.2f) cov=(%.3f,%.3f,%.3f)\033[0m",
    name_.c_str(),
    sym.chr(),
    sym.index(),
    gps_x,
    gps_y,
    gps_z,
    noise_x,
    noise_y,
    noise_z);

  return result;
}

// ==========================================================================
// TF + Callbacks
// ==========================================================================

void GpsFactor::broadcastUtmToMap()
{
  if (!static_tf_broadcaster_) return;

  Eigen::Matrix3d R_enu_map = R_map_enu_.transpose();
  Eigen::Vector3d offset_vec(utm_to_map_offset_.x(), utm_to_map_offset_.y(), utm_to_map_offset_.z());
  Eigen::Vector3d t_tf = R_enu_map * offset_vec;
  Eigen::Quaterniond q_enu_map(R_enu_map);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = utm_frame_;
  tf.child_frame_id = map_frame_;
  tf.transform.translation.x = t_tf.x();
  tf.transform.translation.y = t_tf.y();
  tf.transform.translation.z = t_tf.z();
  tf.transform.rotation.x = q_enu_map.x();
  tf.transform.rotation.y = q_enu_map.y();
  tf.transform.rotation.z = q_enu_map.z();
  tf.transform.rotation.w = q_enu_map.w();

  static_tf_broadcaster_->sendTransform(tf);
}

void GpsFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  if (q.length2() < kQuatLength2Min) return;

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  std::lock_guard<std::mutex> lock(imu_orientation_lock_);
  latest_imu_yaw_ = yaw;
  has_imu_orientation_.store(true, std::memory_order_relaxed);
}

Eigen::Vector3d GpsFactor::utmToMap(const Eigen::Vector3d & utm_pos) const
{
  return R_map_enu_ * utm_pos;
}

void GpsFactor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
  while (gps_queue_.size() > 100) gps_queue_.pop_front();

  if (active_) {
    UtmCoordinate utm = latLonToUtm(msg->latitude, msg->longitude, msg->altitude);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = utm_frame_;
    pose.pose.position.x = utm.easting;
    pose.pose.position.y = utm.northing;
    pose.pose.position.z = utm.altitude;
    {
      std::lock_guard<std::mutex> imu_lock(imu_orientation_lock_);
      tf2::Quaternion q;
      q.setRPY(0, 0, latest_imu_yaw_);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    }
    if (utm_pub_->is_activated()) utm_pub_->publish(pose);
  }
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsFactor, eidos::FactorPlugin)
