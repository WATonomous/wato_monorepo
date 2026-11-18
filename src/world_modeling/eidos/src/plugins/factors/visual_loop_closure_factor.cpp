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

#include "eidos/plugins/factors/visual_loop_closure_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

namespace
{

constexpr double kGiveUpSec = 10.0;  // wall time a keyframe may wait for its images
constexpr double kLogPeriodSec = 10.0;

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

double angleDiff(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * M_PI));
}

}  // namespace

// ==========================================================================
// Lifecycle
// ==========================================================================

VisualLoopClosureFactor::~VisualLoopClosureFactor()
{
  deactivate();
}

void VisualLoopClosureFactor::onInitialize()
{
  const std::string & prefix = name_;
  double max_heading_diff_deg = max_heading_diff_ * 180.0 / M_PI;

  node_->declare_parameter(prefix + ".add_factors", add_factors_);
  node_->declare_parameter(prefix + ".shadow_odometry", shadow_odometry_);
  node_->declare_parameter(prefix + ".candidate_mode", candidate_mode_);
  node_->declare_parameter(prefix + ".search_radius", search_radius_);
  node_->declare_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->declare_parameter(prefix + ".max_heading_diff_deg", max_heading_diff_deg);
  node_->declare_parameter(prefix + ".min_similarity", min_similarity_);
  node_->declare_parameter(prefix + ".max_frame_wait", max_frame_wait_);
  node_->declare_parameter(prefix + ".max_queue", max_queue_);
  node_->declare_parameter(prefix + ".min_cameras", min_cameras_);
  node_->declare_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->declare_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->declare_parameter(prefix + ".gate_trans", gate_trans_);
  node_->declare_parameter(prefix + ".gate_rot", gate_rot_);
  node_->declare_parameter(prefix + ".gate_place_recognition", gate_place_recognition_);
  node_->declare_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->declare_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->declare_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->declare_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->declare_parameter(prefix + ".robust_k", robust_k_);

  node_->get_parameter(prefix + ".add_factors", add_factors_);
  node_->get_parameter(prefix + ".shadow_odometry", shadow_odometry_);
  node_->get_parameter(prefix + ".candidate_mode", candidate_mode_);
  node_->get_parameter(prefix + ".search_radius", search_radius_);
  node_->get_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->get_parameter(prefix + ".max_heading_diff_deg", max_heading_diff_deg);
  node_->get_parameter(prefix + ".min_similarity", min_similarity_);
  node_->get_parameter(prefix + ".max_frame_wait", max_frame_wait_);
  node_->get_parameter(prefix + ".max_queue", max_queue_);
  node_->get_parameter(prefix + ".min_cameras", min_cameras_);
  node_->get_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->get_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->get_parameter(prefix + ".gate_trans", gate_trans_);
  node_->get_parameter(prefix + ".gate_rot", gate_rot_);
  node_->get_parameter(prefix + ".gate_place_recognition", gate_place_recognition_);
  node_->get_parameter(prefix + ".rot_sigma", rot_sigma_);
  node_->get_parameter(prefix + ".trans_sigma", trans_sigma_);
  node_->get_parameter(prefix + ".trans_sigma_per_m", trans_sigma_per_m_);
  node_->get_parameter(prefix + ".robust_kernel", robust_kernel_);
  node_->get_parameter(prefix + ".robust_k", robust_k_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  max_heading_diff_ = max_heading_diff_deg * M_PI / 180.0;

  if (candidate_mode_ != "distance" && candidate_mode_ != "place_recognition" && candidate_mode_ != "both") {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] unknown candidate_mode '%s' (distance|place_recognition|both); using distance",
      name_.c_str(),
      candidate_mode_.c_str());
    candidate_mode_ = "distance";
  }
  if (robust_kernel_ != "none" && robust_kernel_ != "huber" && robust_kernel_ != "cauchy") {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] unknown robust_kernel '%s' (none|huber|cauchy); using huber",
      name_.c_str(),
      robust_kernel_.c_str());
    robust_kernel_ = "huber";
  }

  frames_key_ = name_ + "/frames";
  descriptors_key_ = name_ + "/descriptors";
  map_manager_->registerKeyframeFormat(frames_key_, "visual_frame_set");
  map_manager_->registerKeyframeFormat(descriptors_key_, "raw_float_vector");

  keyframes_ = std::make_unique<VisualKeyframes>(node_, name_, callback_group_);
  measurement_pub_ =
    node_->create_publisher<eidos_msgs::msg::LoopClosureMeasurement>(name_ + "/measurement", rclcpp::QoS(100));

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (visual loop closure, candidates=%s, add_factors=%d, shadow_odometry=%d)",
    name_.c_str(),
    candidate_mode_.c_str(),
    add_factors_,
    shadow_odometry_);
}

void VisualLoopClosureFactor::activate()
{
  if (running_.exchange(true)) return;
  active_ = true;
  measurement_pub_->on_activate();
  keyframes_->activate();
  record_thread_ = std::thread(&VisualLoopClosureFactor::recordLoop, this);
  measure_thread_ = std::thread(&VisualLoopClosureFactor::measureLoop, this);
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void VisualLoopClosureFactor::deactivate()
{
  if (!running_.exchange(false)) return;
  active_ = false;
  queue_cv_.notify_all();
  if (record_thread_.joinable()) record_thread_.join();
  if (measure_thread_.joinable()) measure_thread_.join();
  keyframes_->deactivate();
  measurement_pub_->on_deactivate();
  {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    pending_.clear();
    recorded_.clear();
    results_.clear();
  }
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ==========================================================================
// SLAM loop
// ==========================================================================

StampedFactorResult VisualLoopClosureFactor::latchFactor(gtsam::Key key, double timestamp)
{
  StampedFactorResult result;
  if (!active_) return result;
  if (state_->load(std::memory_order_acquire) != SlamState::TRACKING) return result;

  {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    for (const auto & c : results_) {
      result.factors.push_back(
        gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(c.from, c.to, c.measured, c.noise));
      gtsam::Symbol from_sym(c.from), to_sym(c.to);
      RCLCPP_INFO(
        node_->get_logger(),
        "\033[31m[%s] DELIVERING loop closure: (%c,%lu)->(%c,%lu) t=(%.2f,%.2f,%.2f)\033[0m",
        name_.c_str(),
        from_sym.chr(),
        from_sym.index(),
        to_sym.chr(),
        to_sym.index(),
        c.measured.x(),
        c.measured.y(),
        c.measured.z());
    }
    result.loop_closure = !results_.empty();
    results_.clear();
    pending_.push_back({key, timestamp, std::chrono::steady_clock::now()});
  }
  queue_cv_.notify_all();
  logCounters();
  return result;
}

// ==========================================================================
// Recording
// ==========================================================================

void VisualLoopClosureFactor::recordLoop()
{
  while (running_) {
    Pending pending;
    {
      std::unique_lock<std::mutex> lock(queue_mtx_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [this] { return !running_ || !pending_.empty(); });
      if (!running_) return;
      if (pending_.empty()) continue;
      pending = pending_.front();
    }
    const RecordStatus status = record(pending);
    if (status == RecordStatus::kWait) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      pending_.pop_front();
      if (status == RecordStatus::kRecorded) {
        recorded_.push_back(pending.key);
        while (static_cast<int>(recorded_.size()) > max_queue_) {
          recorded_.pop_front();  // the pose service cannot keep up: skip the oldest keyframes
          ++counters_.dropped;
        }
      }
    }
    queue_cv_.notify_all();
  }
}

VisualLoopClosureFactor::RecordStatus VisualLoopClosureFactor::record(const Pending & pending)
{
  const bool gave_up =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - pending.queued).count() > kGiveUpSec;

  // The keyframe's nearest images are known once every camera has delivered a later one (or has
  // been silent for max_frame_wait), and its pose once the SLAM loop has added it to the map.
  const auto newest = keyframes_->newestStamp();
  const bool images_known = keyframes_->covers(pending.stamp) || (newest && *newest - pending.stamp > max_frame_wait_);
  const bool in_map = map_manager_->getCloudIndex(pending.key) >= 0;
  if ((!images_known || !in_map) && !gave_up) return RecordStatus::kWait;

  auto frames = keyframes_->framesNear(pending.stamp);
  if (!frames || !in_map) {
    ++counters_.no_images;
    return RecordStatus::kSkipped;
  }
  map_manager_->store(pending.key, frames_key_, std::move(*frames));
  ++counters_.recorded;
  return RecordStatus::kRecorded;
}

// ==========================================================================
// Measuring
// ==========================================================================

void VisualLoopClosureFactor::measureLoop()
{
  const bool need_descriptors = candidate_mode_ != "distance";
  while (running_) {
    if (!keyframes_->servicesReady(need_descriptors)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        10000,
        "[%s] waiting for the pose%s service; keyframes are being recorded",
        name_.c_str(),
        need_descriptors ? " and descriptor" : "");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    gtsam::Key key;
    {
      std::unique_lock<std::mutex> lock(queue_mtx_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return !running_ || !recorded_.empty(); });
      if (!running_) return;
      if (recorded_.empty()) continue;
      key = recorded_.front();
      recorded_.pop_front();
    }
    process(key);
  }
}

void VisualLoopClosureFactor::process(gtsam::Key key)
{
  const bool use_distance = candidate_mode_ != "place_recognition";
  const bool use_place = candidate_mode_ != "distance";

  if (use_place) {
    auto frames = map_manager_->retrieve<VisualFrameSet>(key, frames_key_);
    std::string error;
    std::size_t dim = 0;
    auto descriptors = frames ? keyframes_->describe(*frames, dim, error) : std::nullopt;
    if (descriptors) {
      descriptor_dim_ = dim;
      descriptors_[key] = std::make_shared<const std::vector<float>>(*descriptors);
      map_manager_->store(key, descriptors_key_, std::move(*descriptors));
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "[%s] no descriptors: %s", name_.c_str(), error.c_str());
    }
  }

  if (shadow_odometry_ && last_processed_) measure(*last_processed_, key, "consecutive", 0.0);
  last_processed_ = key;

  std::optional<gtsam::Key> by_distance = use_distance ? distanceCandidate(key) : std::nullopt;
  if (by_distance) measure(*by_distance, key, "distance", 0.0);
  if (use_place) {
    double similarity = 0.0;
    const auto by_place = placeCandidate(key, similarity);
    if (by_place && by_place != by_distance) measure(*by_place, key, "place_recognition", similarity);
  }
}

std::optional<gtsam::Key> VisualLoopClosureFactor::distanceCandidate(gtsam::Key key) const
{
  const auto poses = map_manager_->getKeyPoses6D();
  const int idx = map_manager_->getCloudIndex(key);
  if (idx < 0 || idx >= static_cast<int>(poses->size())) return std::nullopt;
  const auto & self = poses->points[idx];

  std::optional<gtsam::Key> best;
  double best_dist = search_radius_;
  for (int i = 0; i < static_cast<int>(poses->size()); ++i) {
    const auto & other = poses->points[i];
    if (i == idx || std::abs(self.time - other.time) < search_time_diff_) continue;
    const double dist = std::hypot(self.x - other.x, self.y - other.y, self.z - other.z);
    if (dist >= best_dist || angleDiff(self.yaw, other.yaw) > max_heading_diff_) continue;
    const gtsam::Key candidate = map_manager_->getKeyFromCloudIndex(i);
    // Prior-map keyframes are not in the graph; keyframes without images cannot be measured.
    if (candidate == 0 || map_manager_->isPriorMapKey(candidate)) continue;
    if (!map_manager_->hasKeyframeData(candidate, frames_key_)) continue;
    best = candidate;
    best_dist = dist;
  }
  return best;
}

std::optional<gtsam::Key> VisualLoopClosureFactor::placeCandidate(gtsam::Key key, double & similarity) const
{
  const auto self_it = descriptors_.find(key);
  if (self_it == descriptors_.end()) return std::nullopt;
  const auto poses = map_manager_->getKeyPoses6D();
  const int idx = map_manager_->getCloudIndex(key);
  if (idx < 0 || idx >= static_cast<int>(poses->size())) return std::nullopt;
  const double self_time = poses->points[idx].time;

  std::optional<gtsam::Key> best;
  similarity = min_similarity_;
  for (const auto & [candidate, descriptors] : descriptors_) {
    if (candidate == key) continue;
    const int i = map_manager_->getCloudIndex(candidate);
    if (i < 0 || i >= static_cast<int>(poses->size())) continue;
    if (std::abs(self_time - poses->points[i].time) < search_time_diff_) continue;
    const double score = placeSimilarity(*self_it->second, *descriptors, descriptor_dim_);
    if (score > similarity) {
      similarity = score;
      best = candidate;
    }
  }
  return best;
}

gtsam::SharedNoiseModel VisualLoopClosureFactor::noiseFor(double distance) const
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

void VisualLoopClosureFactor::measure(
  gtsam::Key from, gtsam::Key to, const std::string & candidate_source, double similarity)
{
  const auto frames_from = map_manager_->retrieve<VisualFrameSet>(from, frames_key_);
  const auto frames_to = map_manager_->retrieve<VisualFrameSet>(to, frames_key_);
  const auto poses = map_manager_->getKeyPoses6D();
  const int from_idx = map_manager_->getCloudIndex(from);
  const int to_idx = map_manager_->getCloudIndex(to);
  const int n = static_cast<int>(poses->size());
  if (!frames_from || !frames_to || from_idx < 0 || to_idx < 0 || from_idx >= n || to_idx >= n) return;

  std::string error;
  const auto estimate = keyframes_->estimateRelativePose(*frames_from, *frames_to, error);
  if (!estimate) {
    ++counters_.failed;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "[%s] pose service: %s", name_.c_str(), error.c_str());
    return;
  }
  ++counters_.measured;

  // The service measures between the image times; move both ends to the keyframe times.
  const gtsam::Pose3 measured = motionSinceKeyframe(*map_manager_, from, frames_from->stamp) * estimate->a_from_b *
                                motionSinceKeyframe(*map_manager_, to, frames_to->stamp).inverse();
  const gtsam::Pose3 graph =
    poseTypeToGtsamPose3(poses->points[from_idx]).between(poseTypeToGtsamPose3(poses->points[to_idx]));
  const gtsam::Pose3 residual = measured.between(graph);
  const double res_trans = residual.translation().norm();
  const double res_rot = gtsam::Rot3::Logmap(residual.rotation()).norm();

  // Place recognition exists to find loops the drifted graph no longer puts close together, so by
  // default its candidates are not held to agreeing with the graph.
  const bool gated = candidate_source != "place_recognition" || gate_place_recognition_;
  std::string reject;
  if (static_cast<int>(estimate->cameras) < min_cameras_) {
    reject = "too few cameras";
  } else if (estimate->trans_spread > max_trans_spread_ || estimate->rot_spread > max_rot_spread_) {
    reject = "cameras disagree";
  } else if (gated && ((gate_trans_ > 0.0 && res_trans > gate_trans_) || (gate_rot_ > 0.0 && res_rot > gate_rot_))) {
    reject = "disagrees with graph";
  }
  const bool consecutive = candidate_source == "consecutive";
  const bool accepted = reject.empty();
  const bool added = accepted && add_factors_ && !consecutive;

  eidos_msgs::msg::LoopClosureMeasurement msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(poses->points[to_idx].time * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = base_link_frame_;
  msg.source = name_;
  msg.candidate_source = candidate_source;
  msg.from_index = gtsam::Symbol(from).index();
  msg.to_index = gtsam::Symbol(to).index();
  msg.from_stamp = poses->points[from_idx].time;
  msg.to_stamp = poses->points[to_idx].time;
  msg.measured = toMsg(measured);
  msg.graph = toMsg(graph);
  msg.accepted = accepted;
  msg.added = added;
  msg.reject_reason = reject;
  msg.trans_spread = estimate->trans_spread;
  msg.rot_spread = estimate->rot_spread;
  msg.confidence = estimate->confidence;
  msg.cameras = estimate->cameras;
  msg.similarity = similarity;
  msg.duration = estimate->duration;
  measurement_pub_->publish(msg);

  if (consecutive) return;
  if (!accepted) {
    ++counters_.rejected;
    return;
  }
  ++counters_.accepted;
  RCLCPP_INFO(
    node_->get_logger(),
    "\033[31m[%s] %s (x,%lu)->(x,%lu) via %s: t=(%.2f,%.2f,%.2f), %.2f m / %.2f deg from the graph, "
    "spread %.2f m, %.0f ms\033[0m",
    name_.c_str(),
    added ? "LOOP CLOSURE" : "loop closure (shadow)",
    msg.from_index,
    msg.to_index,
    candidate_source.c_str(),
    measured.x(),
    measured.y(),
    measured.z(),
    res_trans,
    res_rot * 180.0 / M_PI,
    estimate->trans_spread,
    estimate->duration * 1e3);
  if (added) {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    results_.push_back({from, to, measured, noiseFor(measured.translation().norm())});
  }
}

void VisualLoopClosureFactor::logCounters()
{
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_log_).count() < kLogPeriodSec) return;
  last_log_ = now;
  std::size_t waiting = 0;
  {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    waiting = recorded_.size();
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] keyframes recorded=%zu no_images=%zu dropped=%zu waiting=%zu | measured=%zu failed=%zu | "
    "loops %s=%zu rejected=%zu",
    name_.c_str(),
    counters_.recorded.load(),
    counters_.no_images.load(),
    counters_.dropped.load(),
    waiting,
    counters_.measured.load(),
    counters_.failed.load(),
    add_factors_ ? "added" : "shadow",
    counters_.accepted.load(),
    counters_.rejected.load());
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::VisualLoopClosureFactor, eidos::FactorPlugin)
