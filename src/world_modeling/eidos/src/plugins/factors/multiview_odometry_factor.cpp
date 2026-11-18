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

#include "eidos/plugins/factors/multiview_odometry_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/biased_relative_pose_factor.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos
{

namespace
{

constexpr double kLogPeriodSec = 10.0;
constexpr double kGpsHistorySec = 120.0;
/// When a camera has no image near the chosen time, the next attempt is this much later (s).
constexpr double kRetryStepSec = 0.05;

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

geometry_msgs::msg::Pose toMsg(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  const auto q = pose.rotation().toQuaternion();
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  msg.position.z = pose.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

void sleepMs(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

gtsam::Key MultiViewOdometryFactor::biasKey(gtsam::Key state)
{
  return gtsam::Symbol('c', gtsam::Symbol(state).index());
}

// ==========================================================================
// Lifecycle
// ==========================================================================

MultiViewOdometryFactor::~MultiViewOdometryFactor()
{
  running_ = false;
  if (worker_.joinable()) worker_.join();
}

void MultiViewOdometryFactor::onInitialize()
{
  const std::string prefix = name_;
  std::string gps_topic;
  int max_ready = static_cast<int>(max_ready_);

  node_->declare_parameter(prefix + ".gps_topic", std::string("/novatel/oem7/fix"));
  node_->declare_parameter(prefix + ".time_offset", time_offset_);
  node_->declare_parameter(prefix + ".depth.enabled", false);
  node_->declare_parameter(prefix + ".keyframe_distance", keyframe_distance_);
  node_->declare_parameter(prefix + ".keyframe_max_interval", keyframe_max_interval_);
  node_->declare_parameter(prefix + ".add_factors", add_factors_);
  node_->declare_parameter(prefix + ".create_without_measurement", create_without_measurement_);
  node_->declare_parameter(prefix + ".cloud.enabled", true);
  node_->declare_parameter(prefix + ".cloud.voxel", 0.2);
  node_->declare_parameter(prefix + ".cloud.max_range", 40.0);
  node_->declare_parameter(prefix + ".cloud.every_n", cloud_every_n_);
  node_->declare_parameter(prefix + ".min_cameras", min_cameras_);
  node_->declare_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->declare_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->declare_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->declare_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->declare_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->declare_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->declare_parameter(prefix + ".bias.enabled", bias_enabled_);
  node_->declare_parameter(prefix + ".bias.prior_sigma", bias_prior_sigma_);
  node_->declare_parameter(prefix + ".bias.random_walk", bias_random_walk_);
  node_->declare_parameter(prefix + ".robust_k", robust_k_);
  node_->declare_parameter(prefix + ".max_queue", max_ready);

  double cloud_voxel = 0.2, cloud_max_range = 40.0;
  bool depth_enabled = false;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".time_offset", time_offset_);
  node_->get_parameter(prefix + ".depth.enabled", depth_enabled);
  node_->get_parameter(prefix + ".keyframe_distance", keyframe_distance_);
  node_->get_parameter(prefix + ".keyframe_max_interval", keyframe_max_interval_);
  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter(prefix + ".create_without_measurement", create_without_measurement_);
  node_->get_parameter(prefix + ".cloud.enabled", cloud_request_.enabled);
  node_->get_parameter(prefix + ".cloud.voxel", cloud_voxel);
  node_->get_parameter(prefix + ".cloud.max_range", cloud_max_range);
  node_->get_parameter(prefix + ".cloud.every_n", cloud_every_n_);
  node_->get_parameter(prefix + ".min_cameras", min_cameras_);
  node_->get_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->get_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->get_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->get_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->get_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->get_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->get_parameter(prefix + ".bias.enabled", bias_enabled_);
  node_->get_parameter(prefix + ".bias.prior_sigma", bias_prior_sigma_);
  node_->get_parameter(prefix + ".bias.random_walk", bias_random_walk_);
  if (bias_prior_sigma_.size() != 3 || bias_random_walk_.size() != 3) {
    throw std::invalid_argument(name_ + ": bias.prior_sigma and bias.random_walk need three values");
  }
  node_->get_parameter(prefix + ".robust_k", robust_k_);
  node_->get_parameter(prefix + ".max_queue", max_ready);
  node_->get_parameter("frames.base_link", base_link_frame_);
  cloud_request_.voxel = static_cast<float>(cloud_voxel);
  cloud_request_.max_range = static_cast<float>(cloud_max_range);
  max_ready_ = static_cast<std::size_t>(std::max(1, max_ready));
  cloud_every_n_ = std::max(1, cloud_every_n_);
  if (keyframe_max_interval_ <= 0.0 && gps_topic.empty()) {
    throw std::invalid_argument(name_ + ": needs a gps_topic or a keyframe_max_interval to choose keyframes");
  }

  keyframes_ = std::make_unique<VisualKeyframes>(node_, prefix, callback_group_);
  if (depth_enabled) depth_ = std::make_unique<LidarDepth>(node_, prefix, callback_group_, tf_);

  if (!gps_topic.empty()) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;
    gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&MultiViewOdometryFactor::gpsCallback, this, std::placeholders::_1),
      sub_opts);
  }
  measurement_pub_ = node_->create_publisher<eidos_msgs::msg::LoopClosureMeasurement>(name_ + "/measurement", 100);
  bias_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(name_ + "/bias", 100);

  cloud_key_ = name_ + "/cloud";
  map_manager_->registerKeyframeFormat(cloud_key_, "pcl_xyzrgb_binary");

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (keyframes every %.1f m on %s or %.1f s; add_factors=%d, create_without_measurement=%d, "
    "cloud=%d, lidar depth=%d, camera time offset %.3f s, bias states=%d)",
    name_.c_str(),
    keyframe_distance_,
    gps_topic.empty() ? "(no GPS)" : gps_topic.c_str(),
    keyframe_max_interval_,
    add_factors_,
    create_without_measurement_,
    cloud_request_.enabled,
    depth_ != nullptr,
    time_offset_,
    bias_enabled_);
}

void MultiViewOdometryFactor::activate()
{
  if (running_.exchange(true)) return;
  measurement_pub_->on_activate();
  bias_pub_->on_activate();
  keyframes_->activate();
  if (depth_) depth_->setActive(true);
  worker_ = std::thread(&MultiViewOdometryFactor::workLoop, this);
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void MultiViewOdometryFactor::deactivate()
{
  if (!running_.exchange(false)) return;
  tracking_ = false;
  if (worker_.joinable()) worker_.join();
  keyframes_->deactivate();
  if (depth_) depth_->setActive(false);
  measurement_pub_->on_deactivate();
  bias_pub_->on_deactivate();
  {
    std::lock_guard<std::mutex> lock(ready_mtx_);
    ready_.clear();
  }
  pending_.clear();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void MultiViewOdometryFactor::onTrackingBegin(const gtsam::Pose3 & /*pose*/)
{
  // Runs on the SLAM thread before the worker may choose keyframes for this tracking session.
  tracking_ = false;
  {
    std::lock_guard<std::mutex> lock(ready_mtx_);
    ready_.clear();
    restart_ = true;
  }
  prev_key_.reset();
  prev_pose_.reset();
  bias_ = gtsam::Vector3::Zero();
  pending_.clear();
  tracking_ = true;
}

// ==========================================================================
// Choosing and measuring keyframes (worker thread)
// ==========================================================================

void MultiViewOdometryFactor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
  const auto utm = latLonToUtm(msg->latitude, msg->longitude, msg->altitude);
  const double t = toSec(msg->header.stamp);
  std::lock_guard<std::mutex> lock(gps_mtx_);
  if (!gps_.empty() && t <= gps_.back().first) {
    if (t < gps_.back().first - 1.0) gps_.clear();  // the clock restarted (bag replayed)
    else return;
  }
  gps_.emplace_back(t, Eigen::Vector2d(utm.easting, utm.northing));
  while (gps_.size() > 2 && gps_.back().first - gps_.front().first > kGpsHistorySec) gps_.pop_front();
}

std::optional<double> MultiViewOdometryFactor::nextKeyframeTime(double after, double not_before) const
{
  const double limit = keyframe_max_interval_ > 0.0 ? std::max(after + keyframe_max_interval_, not_before)
                                                    : std::numeric_limits<double>::infinity();
  std::lock_guard<std::mutex> lock(gps_mtx_);
  // GPS position at `after` (the first fix after it if none is before)
  std::optional<Eigen::Vector2d> start;
  for (std::size_t i = 0; i < gps_.size(); ++i) {
    if (gps_[i].first < after) continue;
    if (i == 0 || gps_[i].first == after) {
      start = gps_[i].second;
    } else {
      const auto & [ta, pa] = gps_[i - 1];
      const auto & [tb, pb] = gps_[i];
      start = pa + (after - ta) / (tb - ta) * (pb - pa);
    }
    break;
  }
  if (start) {
    for (const auto & [t, p] : gps_) {
      if (t < not_before || t <= after) continue;
      if (t >= limit) break;
      if ((p - *start).norm() >= keyframe_distance_) return t;
    }
  }
  if (std::isfinite(limit)) return limit;
  return std::nullopt;
}

bool MultiViewOdometryFactor::check(const VisualPoseEstimate & estimate, std::string & reason) const
{
  if (static_cast<int>(estimate.cameras) < min_cameras_) {
    reason = "too few cameras";
  } else if (estimate.trans_spread > max_trans_spread_ || estimate.rot_spread > max_rot_spread_) {
    reason = "cameras disagree";
  } else {
    return true;
  }
  return false;
}

void MultiViewOdometryFactor::workLoop()
{
  double not_before = 0.0;  // earliest time for the next keyframe (after a frame set was missing)
  while (running_) {
    if (restart_.exchange(false)) {
      anchor_.reset();
      anchor_depth_.clear();
      not_before = 0.0;
    }
    if (!tracking_) {
      sleepMs(50);
      continue;
    }
    if (!keyframes_->servicesReady(false)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 10000, "[%s] waiting for the pose service", name_.c_str());
      sleepMs(200);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(ready_mtx_);
      if (ready_.size() >= max_ready_) {
        // The SLAM loop is not taking keyframes (e.g. not tracking yet): do not run ahead of it.
        sleepMs(20);
        continue;
      }
    }

    std::optional<VisualFrameSet> frames;
    if (!anchor_) {
      frames = keyframes_->latestFrames();
      if (!frames) {
        sleepMs(20);
        continue;
      }
    } else {
      const auto t = nextKeyframeTime(anchor_->stamp, not_before);
      if (!t || !keyframes_->covers(*t)) {
        sleepMs(10);
        continue;
      }
      frames = keyframes_->framesNear(*t);
      if (!frames || frames->stamp <= anchor_->stamp) {
        ++counters_.no_images;  // a camera has no image near t: try a little later
        not_before = *t + kRetryStepSec;
        continue;
      }
    }

    // Lidar depth for the new images, once the lidar has fired past all of them
    std::vector<sensor_msgs::msg::PointCloud2> depth;
    if (depth_) {
      double newest = frames->stamp;
      for (const auto & f : frames->frames) newest = std::max(newest, f->stamp);
      const auto waited = std::chrono::steady_clock::now();
      while (running_ && !depth_->covers(newest + time_offset_) &&
             std::chrono::steady_clock::now() - waited < std::chrono::seconds(2))
      {
        sleepMs(10);
      }
      LidarDepth::Stats stats;
      depth = depth_->clouds(*frames, time_offset_, stats);
      counters_.depth_points += stats.points;
      ++counters_.depth_sets;
      if (!stats.deskewed) ++counters_.not_deskewed;
    }

    Keyframe keyframe;
    keyframe.frames = *frames;
    if (anchor_) {
      std::string error;
      // The dense points cost about three times the pose alone: only every cloud.every_n keyframes.
      VisualCloudRequest cloud = cloud_request_;
      cloud.enabled = cloud.enabled && counters_.keyframes % static_cast<std::size_t>(cloud_every_n_) == 0;
      keyframe.estimate = keyframes_->estimateRelativePose(*anchor_, *frames, error, cloud, {anchor_depth_, depth});
      if (!keyframe.estimate) {
        ++counters_.failed;
        keyframe.reject = "pose service: " + error;
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000, "[%s] %s", name_.c_str(), keyframe.reject.c_str());
      } else {
        ++counters_.measured;
        if (!check(*keyframe.estimate, keyframe.reject)) {
          ++counters_.rejected;
        } else if (keyframe.estimate->cloud && keyframe.estimate->cloud->width > 0) {
          keyframe.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
          pcl::fromROSMsg(*keyframe.estimate->cloud, *keyframe.cloud);
        }
        keyframe.estimate->cloud.reset();  // kept in keyframe.cloud
      }
      const bool usable = keyframe.estimate && keyframe.reject.empty();
      if (!usable && add_factors_ && !create_without_measurement_) {
        // Nothing would constrain this state: measure the next keyframe from the previous one.
        ++counters_.dropped;
        not_before = frames->stamp + kRetryStepSec;
        continue;
      }
    }

    anchor_ = keyframe.frames;
    anchor_depth_ = std::move(depth);
    not_before = 0.0;
    ++counters_.keyframes;
    std::lock_guard<std::mutex> lock(ready_mtx_);
    if (restart_) continue;  // tracking restarted while measuring
    ready_.push_back(std::move(keyframe));
  }
}

// ==========================================================================
// SLAM loop
// ==========================================================================

gtsam::SharedNoiseModel MultiViewOdometryFactor::noiseFor(double distance) const
{
  const double st = trans_sigma_ + trans_sigma_per_m_ * distance;
  // GTSAM Pose3 tangent order: [rot, trans]
  auto diagonal = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << rot_sigma_, rot_sigma_, rot_sigma_, st, st, st).finished());
  if (robust_kernel_ == "huber") {
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robust_k_), diagonal);
  }
  if (robust_kernel_ == "cauchy") {
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robust_k_), diagonal);
  }
  return diagonal;
}

StampedFactorResult MultiViewOdometryFactor::produceFactor(gtsam::Key key, double /*timestamp*/)
{
  StampedFactorResult result;
  if (!running_ || state_->load(std::memory_order_acquire) != SlamState::TRACKING) return result;

  Keyframe keyframe;
  {
    std::lock_guard<std::mutex> lock(ready_mtx_);
    if (ready_.empty()) return result;
    keyframe = std::move(ready_.front());
    ready_.pop_front();
  }
  const double stamp = keyframe.frames.stamp + time_offset_;  // when the images were taken
  result.timestamp = stamp;

  const bool usable = keyframe.estimate && keyframe.reject.empty();
  const bool with_bias = bias_enabled_ && add_factors_;
  if (with_bias) {
    // Every state gets a bias: a zero-mean prior on the first, a random walk from the previous after
    const gtsam::Key c = biasKey(key);
    result.values.insert(c, gtsam::Vector3(bias_));
    if (!prev_key_) {
      result.factors.push_back(gtsam::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        c,
        gtsam::Vector3::Zero(),
        gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(bias_prior_sigma_[0], bias_prior_sigma_[1], bias_prior_sigma_[2]))));
    } else {
      const double root_dt = std::sqrt(std::max(stamp - prev_stamp_, 1e-3));
      result.factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
        biasKey(*prev_key_),
        c,
        gtsam::Vector3::Zero(),
        gtsam::noiseModel::Diagonal::Sigmas(
          root_dt * gtsam::Vector3(bias_random_walk_[0], bias_random_walk_[1], bias_random_walk_[2]))));
    }
  }
  bool added = false;
  if (prev_key_ && usable && add_factors_) {
    const gtsam::Pose3 & motion = keyframe.estimate->a_from_b;
    const auto noise = noiseFor(motion.translation().norm());
    const gtsam::Pose3 prev = prev_pose_.value_or(estimator_pose_->load().value_or(gtsam::Pose3()));
    if (with_bias) {
      result.factors.push_back(
        gtsam::make_shared<BiasedRelativePoseFactor>(*prev_key_, key, biasKey(key), motion, noise));
      result.values.insert(key, prev * unbiasedRelativePose(motion, bias_));
    } else {
      result.factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(*prev_key_, key, motion, noise));
      result.values.insert(key, prev * motion);
    }
    added = true;
  }
  if (keyframe.cloud) map_manager_->store(key, cloud_key_, keyframe.cloud);
  keyframe.cloud.reset();

  if (prev_key_ && keyframe.estimate) {
    pending_.push_back({*prev_key_, key, prev_stamp_, stamp, std::move(keyframe), added});
  }
  prev_key_ = key;
  prev_stamp_ = stamp;
  prev_pose_.reset();
  logCounters();
  return result;
}

void MultiViewOdometryFactor::onOptimizationComplete(const gtsam::Values & values, bool /*graph_corrected*/)
{
  if (prev_key_ && values.exists(*prev_key_)) prev_pose_ = values.at<gtsam::Pose3>(*prev_key_);
  if (bias_enabled_ && prev_key_ && values.exists(biasKey(*prev_key_))) {
    bias_ = values.at<gtsam::Vector3>(biasKey(*prev_key_));
    if (bias_pub_->is_activated()) {
      geometry_msgs::msg::Vector3Stamped msg;
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(prev_stamp_ * 1e9), RCL_ROS_TIME);
      msg.header.frame_id = base_link_frame_;
      msg.vector.x = bias_(kTranslationScale);
      msg.vector.y = bias_(kYawPerMetre);
      msg.vector.z = bias_(kRotationScale);
      bias_pub_->publish(msg);
    }
  }
  while (!pending_.empty() && values.exists(pending_.front().from) && values.exists(pending_.front().to)) {
    publish(pending_.front(), values);
    pending_.pop_front();
  }
}

void MultiViewOdometryFactor::publish(const Pending & pending, const gtsam::Values & values)
{
  const auto & estimate = *pending.keyframe.estimate;
  eidos_msgs::msg::LoopClosureMeasurement msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(pending.to_stamp * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = base_link_frame_;
  msg.source = name_;
  msg.candidate_source = "consecutive";
  msg.from_index = gtsam::Symbol(pending.from).index();
  msg.to_index = gtsam::Symbol(pending.to).index();
  msg.from_stamp = pending.from_stamp;
  msg.to_stamp = pending.to_stamp;
  msg.measured = toMsg(estimate.a_from_b);
  msg.graph = toMsg(values.at<gtsam::Pose3>(pending.from).between(values.at<gtsam::Pose3>(pending.to)));
  msg.accepted = pending.keyframe.reject.empty();
  msg.added = pending.added;
  msg.reject_reason = pending.keyframe.reject;
  msg.trans_spread = estimate.trans_spread;
  msg.rot_spread = estimate.rot_spread;
  msg.confidence = estimate.confidence;
  msg.cameras = estimate.cameras;
  msg.duration = estimate.duration;
  measurement_pub_->publish(msg);
}

void MultiViewOdometryFactor::logCounters()
{
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_log_).count() < kLogPeriodSec) return;
  last_log_ = now;
  std::size_t waiting = 0;
  {
    std::lock_guard<std::mutex> lock(ready_mtx_);
    waiting = ready_.size();
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] keyframes=%zu waiting=%zu | measured=%zu failed=%zu rejected=%zu dropped=%zu no_images=%zu (%s) | "
    "lidar points per keyframe=%zu not_deskewed=%zu | bias: scale %+.2f %%, yaw %+.3f deg/100 m, rotation %+.2f %%",
    name_.c_str(),
    counters_.keyframes.load(),
    waiting,
    counters_.measured.load(),
    counters_.failed.load(),
    counters_.rejected.load(),
    counters_.dropped.load(),
    counters_.no_images.load(),
    add_factors_ ? "factors added" : "shadow",
    counters_.depth_sets.load() ? counters_.depth_points.load() / counters_.depth_sets.load() : 0,
    counters_.not_deskewed.load(),
    100.0 * bias_(kTranslationScale),
    bias_(kYawPerMetre) * 180.0 / M_PI * 100.0,
    100.0 * bias_(kRotationScale));
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::MultiViewOdometryFactor, eidos::FactorPlugin)
