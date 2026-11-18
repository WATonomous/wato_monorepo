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

#include "eidos/plugins/relocalization/visual_relocalization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"

namespace eidos
{

VisualRelocalization::~VisualRelocalization()
{
  if (thread_.joinable()) thread_.join();
}

void VisualRelocalization::onInitialize()
{
  const std::string & prefix = name_;
  node_->declare_parameter(prefix + ".keyframes_from", keyframes_from_);
  node_->declare_parameter(prefix + ".top_k", top_k_);
  node_->declare_parameter(prefix + ".min_similarity", min_similarity_);
  node_->declare_parameter(prefix + ".agree_trans", agree_trans_);
  node_->declare_parameter(prefix + ".agree_rot", agree_rot_);
  node_->declare_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->declare_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->declare_parameter(prefix + ".min_interval", min_interval_);
  node_->declare_parameter(prefix + ".max_speed", max_speed_);
  node_->declare_parameter(prefix + ".max_age", max_age_);

  node_->get_parameter(prefix + ".keyframes_from", keyframes_from_);
  node_->get_parameter(prefix + ".top_k", top_k_);
  node_->get_parameter(prefix + ".min_similarity", min_similarity_);
  node_->get_parameter(prefix + ".agree_trans", agree_trans_);
  node_->get_parameter(prefix + ".agree_rot", agree_rot_);
  node_->get_parameter(prefix + ".max_trans_spread", max_trans_spread_);
  node_->get_parameter(prefix + ".max_rot_spread", max_rot_spread_);
  node_->get_parameter(prefix + ".min_interval", min_interval_);
  node_->get_parameter(prefix + ".max_speed", max_speed_);
  node_->get_parameter(prefix + ".max_age", max_age_);
  node_->get_parameter("frames.map", map_frame_);

  keyframes_ = std::make_unique<VisualKeyframes>(node_, name_, callback_group_);
  estimate_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/estimate", 10);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (visual relocalization against keyframes of '%s')",
    name_.c_str(),
    keyframes_from_.c_str());
}

void VisualRelocalization::activate()
{
  active_ = true;
  estimate_pub_->on_activate();
  keyframes_->activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void VisualRelocalization::deactivate()
{
  active_ = false;
  if (thread_.joinable()) thread_.join();
  keyframes_->deactivate();
  estimate_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

std::optional<RelocalizationResult> VisualRelocalization::tryRelocalize(double timestamp)
{
  if (!active_ || !map_manager_->hasPriorMap()) return std::nullopt;

  {
    std::lock_guard<std::mutex> lock(result_mtx_);
    if (result_) {
      const double age = timestamp - result_->stamp;
      if (age >= 0.0 && age <= max_age_) {
        RelocalizationResult out;
        out.pose = result_->pose * gtsam::Pose3::Expmap(result_->twist * age);
        out.fitness_score = result_->spread;
        out.matched_keyframe_index = result_->keyframe_index;
        result_.reset();
        return out;
      }
      result_.reset();  // too old to carry forward; try again
    }
  }

  if (!attempt_running_.exchange(true)) {
    if (thread_.joinable()) thread_.join();
    thread_ = std::thread([this] {
      attempt();
      attempt_running_ = false;
    });
  }
  return std::nullopt;
}

std::vector<VisualRelocalization::Candidate> VisualRelocalization::rank(
  const std::vector<float> & descriptors, std::size_t dim) const
{
  std::vector<Candidate> candidates;
  const auto keys = map_manager_->getKeyList();
  for (gtsam::Key key : keys) {
    if (!map_manager_->isPriorMapKey(key)) continue;
    const auto other = map_manager_->retrieve<std::vector<float>>(key, keyframes_from_ + "/descriptors");
    if (!other || !map_manager_->hasKeyframeData(key, keyframes_from_ + "/frames")) continue;
    const double similarity = placeSimilarity(descriptors, *other, dim);
    if (similarity >= min_similarity_) candidates.push_back({key, map_manager_->getCloudIndex(key), similarity});
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.similarity > b.similarity;
  });
  if (static_cast<int>(candidates.size()) > top_k_) candidates.resize(top_k_);
  return candidates;
}

std::optional<gtsam::Pose3> VisualRelocalization::locate(
  const Candidate & candidate, const VisualFrameSet & live, double & spread)
{
  const auto frames = map_manager_->retrieve<VisualFrameSet>(candidate.key, keyframes_from_ + "/frames");
  const auto poses = map_manager_->getKeyPoses6D();
  if (!frames || candidate.index < 0 || candidate.index >= static_cast<int>(poses->size())) return std::nullopt;

  std::string error;
  const auto estimate = keyframes_->estimateRelativePose(*frames, live, error);
  if (!estimate) {
    RCLCPP_WARN(node_->get_logger(), "[%s] pose service: %s", name_.c_str(), error.c_str());
    return std::nullopt;
  }
  if (estimate->trans_spread > max_trans_spread_ || estimate->rot_spread > max_rot_spread_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] keyframe %d: cameras disagree (%.2f m, %.2f deg)",
      name_.c_str(),
      candidate.index,
      estimate->trans_spread,
      estimate->rot_spread * 180.0 / M_PI);
    return std::nullopt;
  }
  spread = estimate->trans_spread;
  // map <- keyframe <- keyframe at its image time <- live rig at its image time
  return poseTypeToGtsamPose3(poses->points[candidate.index]) *
         motionSinceKeyframe(*map_manager_, candidate.key, frames->stamp) * estimate->a_from_b;
}

void VisualRelocalization::attempt()
{
  if (!keyframes_->servicesReady(true)) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[%s] waiting for the pose and descriptor services",
      name_.c_str());
    return;
  }
  const auto first = keyframes_->latestFrames();
  if (!first) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] waiting for images", name_.c_str());
    return;
  }

  // 1. Which prior keyframes look like this place?
  std::string error;
  std::size_t dim = 0;
  const auto descriptors = keyframes_->describe(*first, dim, error);
  if (!descriptors) {
    RCLCPP_WARN(node_->get_logger(), "[%s] descriptor service: %s", name_.c_str(), error.c_str());
    return;
  }
  const auto candidates = rank(*descriptors, dim);
  if (candidates.size() < 2) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "[%s] %zu prior keyframes look alike enough (need 2, min_similarity %.2f)",
      name_.c_str(),
      candidates.size(),
      min_similarity_);
    return;
  }

  // 2. Two of them must put the vehicle in the same place.
  struct Located
  {
    Candidate candidate;
    gtsam::Pose3 pose;
    double spread;
  };

  std::vector<Located> located;
  std::optional<Located> best;
  for (const auto & candidate : candidates) {
    double spread = 0.0;
    const auto pose = locate(candidate, *first, spread);
    if (!pose) continue;
    for (const auto & other : located) {
      const gtsam::Pose3 diff = other.pose.between(*pose);
      if (diff.translation().norm() <= agree_trans_ && gtsam::Rot3::Logmap(diff.rotation()).norm() <= agree_rot_) {
        best = other.spread <= spread ? other : Located{candidate, *pose, spread};
        break;
      }
    }
    if (best) break;
    located.push_back({candidate, *pose, spread});
  }
  if (!best) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] no two of %zu candidates agree (%zu measured)",
      name_.c_str(),
      candidates.size(),
      located.size());
    return;
  }

  // 3. A newer set of images against the same keyframe gives the velocity.
  std::optional<VisualFrameSet> second;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (active_ && std::chrono::steady_clock::now() < deadline) {
    second = keyframes_->latestFrames();
    if (second && second->stamp >= first->stamp + min_interval_) break;
    second.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Estimate estimate;
  estimate.pose = best->pose;
  estimate.stamp = first->stamp;
  estimate.spread = best->spread;
  estimate.keyframe_index = best->candidate.index;
  if (second) {
    double spread = 0.0;
    const auto pose = locate(best->candidate, *second, spread);
    if (!pose) return;
    const double dt = second->stamp - first->stamp;
    const gtsam::Pose3 motion = best->pose.between(*pose);
    if (motion.translation().norm() > max_speed_ * dt + agree_trans_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] second estimate is %.1f m from the first after %.2f s; rejected",
        name_.c_str(),
        motion.translation().norm(),
        dt);
      return;
    }
    estimate.twist = gtsam::Pose3::Logmap(motion) / dt;
    estimate.pose = *pose;
    estimate.stamp = second->stamp;
    estimate.spread = std::max(estimate.spread, spread);
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(estimate.stamp * 1e9), RCL_ROS_TIME);
  msg.header.frame_id = map_frame_;
  const auto q = estimate.pose.rotation().toQuaternion();
  msg.pose.position.x = estimate.pose.x();
  msg.pose.position.y = estimate.pose.y();
  msg.pose.position.z = estimate.pose.z();
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  estimate_pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "\033[32m[%s] RELOCALIZED against keyframe %d (similarity %.2f): pos=(%.1f,%.1f,%.1f) yaw=%.1f deg, "
    "speed %.1f m/s, spread %.2f m\033[0m",
    name_.c_str(),
    estimate.keyframe_index,
    best->candidate.similarity,
    estimate.pose.x(),
    estimate.pose.y(),
    estimate.pose.z(),
    estimate.pose.rotation().yaw() * 180.0 / M_PI,
    estimate.twist.tail<3>().norm(),
    estimate.spread);

  std::lock_guard<std::mutex> lock(result_mtx_);
  result_ = estimate;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::VisualRelocalization, eidos::RelocalizationPlugin)
