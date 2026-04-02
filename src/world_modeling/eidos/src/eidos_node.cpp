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

#include "eidos/core/eidos_node.hpp"

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "eidos/utils/conversions.hpp"

namespace eidos
{

using gtsam::symbol_shorthand::X;

EidosNode::EidosNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("eidos_node", options)
{
  RCLCPP_INFO(get_logger(), "EidosNode created (unconfigured)");
}

EidosNode::~EidosNode() = default;

// ==========================================================================
// Lifecycle
// ==========================================================================

EidosNode::CallbackReturn EidosNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\033[36m[CONFIGURING]\033[0m ...");

  // TF buffer — shared with plugins for extrinsic lookups
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  // No global "mode" param — plugin behavior is controlled by per-plugin config.
  declare_parameter("slam_rate", 10.0);
  declare_parameter("relocalization_timeout", 30.0);
  declare_parameter("frames.map", std::string("map"));
  declare_parameter("frames.odometry", std::string("odom"));
  declare_parameter("frames.base_link", std::string("base_footprint"));
  declare_parameter("map.load_path", std::string(""));
  declare_parameter("map.save_path", std::string(""));
  declare_parameter("topics.odom_pose_cov", std::vector<double>{1.0, 1.0, 1.0, 0.1, 0.1, 0.1});

  // ISAM2 + prior params (read as locals, passed to GraphOptimizer::configure)
  declare_parameter("isam2.relinearize_threshold", 0.1);
  declare_parameter("isam2.relinearize_skip", 1);
  declare_parameter("isam2.update_iterations", 2);
  declare_parameter("isam2.correction_iterations", 5);
  declare_parameter("isam2.loop_closure_iterations", 20);
  declare_parameter("prior.pose_cov", std::vector<double>{1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2});

  // Plugin list params (read by PluginRegistry)
  declare_parameter("factor_plugins", std::vector<std::string>{});
  declare_parameter("relocalization_plugins", std::vector<std::string>{});
  declare_parameter("visualization_plugins", std::vector<std::string>{});

  // Topic/service names
  declare_parameter("topics.status", std::string("slam/status"));
  declare_parameter("topics.odom", std::string("slam/odometry"));
  declare_parameter("topics.pose", std::string("slam/pose"));
  declare_parameter("topics.save_map_service", std::string("slam/save_map"));
  declare_parameter("topics.load_map_service", std::string("slam/load_map"));

  // Read parameters

  get_parameter("slam_rate", slam_rate_);
  double relocalization_timeout;
  get_parameter("relocalization_timeout", relocalization_timeout);
  get_parameter("frames.map", map_frame_);
  get_parameter("frames.odometry", odom_frame_);
  get_parameter("frames.base_link", base_link_frame_);
  get_parameter("map.load_path", map_load_directory_);
  get_parameter("map.save_path", map_save_directory_);
  get_parameter("topics.odom_pose_cov", odom_pose_cov_);

  // Configure GraphOptimizer (ISAM2 relinearization + prior)
  double relinearize_threshold;
  int relinearize_skip;
  std::vector<double> prior_pose_cov;
  get_parameter("isam2.relinearize_threshold", relinearize_threshold);
  get_parameter("isam2.relinearize_skip", relinearize_skip);
  get_parameter("prior.pose_cov", prior_pose_cov);
  graph_optimizer_.configure(get_logger(), relinearize_threshold, relinearize_skip, prior_pose_cov);
  map_manager_.configure(get_logger());

  // Iteration counts (owned by the node, passed to optimizer per call)
  get_parameter("isam2.update_iterations", update_iterations_);
  get_parameter("isam2.correction_iterations", correction_iterations_);
  get_parameter("isam2.loop_closure_iterations", loop_closure_iterations_);

  // Load plugins via PluginRegistry
  registry_.loadAll(
    shared_from_this(),
    tf_buffer_.get(),
    &map_manager_,
    &graph_optimizer_.getOptimizedPose(),
    &state_,
    &graph_optimizer_.getOptimizedValues());

  // Configure InitSequencer
  init_sequencer_.configure(
    get_logger(), &state_, &registry_, &map_manager_, relocalization_timeout, [this](const gtsam::Pose3 & pose) {
      beginTracking(pose);
    });

  // Publishers
  std::string status_topic, odom_topic, pose_topic;
  get_parameter("topics.status", status_topic);
  get_parameter("topics.odom", odom_topic);
  get_parameter("topics.pose", pose_topic);

  status_pub_ = create_publisher<eidos_msgs::msg::SlamStatus>(status_topic, 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

  // Services
  std::string save_svc, load_svc;
  get_parameter("topics.save_map_service", save_svc);
  get_parameter("topics.load_map_service", load_svc);

  save_map_srv_ = create_service<eidos_msgs::srv::SaveMap>(
    save_svc, std::bind(&EidosNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = create_service<eidos_msgs::srv::LoadMap>(
    load_svc, std::bind(&EidosNode::loadMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "\033[36m[CONFIGURED]\033[0m");
  return CallbackReturn::SUCCESS;
}

EidosNode::CallbackReturn EidosNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATING]\033[0m ...");

  // Load prior map if configured
  if (!map_load_directory_.empty()) {
    if (map_manager_.loadMap(map_load_directory_)) {
      RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATING]\033[0m Loaded prior map from: %s", map_load_directory_.c_str());
    }
  }

  init_sequencer_.reset();
  RCLCPP_INFO(get_logger(), "\033[33m[WARMING_UP]\033[0m Waiting for plugins...");

  // Activate publishers
  status_pub_->on_activate();
  pose_pub_->on_activate();
  odom_pub_->on_activate();

  // Activate all plugins
  registry_.activateAll();

  // Start SLAM timer
  slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto period = std::chrono::duration<double>(1.0 / slam_rate_);
  slam_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EidosNode::tick, this),
    slam_callback_group_);

  RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATED]\033[0m");
  return CallbackReturn::SUCCESS;
}

EidosNode::CallbackReturn EidosNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  slam_timer_->cancel();
  registry_.deactivateAll();

  status_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  odom_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "\033[36m[DEACTIVATED]\033[0m");
  return CallbackReturn::SUCCESS;
}

EidosNode::CallbackReturn EidosNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  slam_timer_.reset();
  tf_buffer_.reset();
  RCLCPP_INFO(get_logger(), "\033[36m[CLEANED UP]\033[0m");
  return CallbackReturn::SUCCESS;
}

EidosNode::CallbackReturn EidosNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\033[36m[SHUTDOWN]\033[0m");
  return CallbackReturn::SUCCESS;
}

// ==========================================================================
// SLAM Loop + State Machine
// ==========================================================================

void EidosNode::tick()
{
  double timestamp = now().seconds();

  // InitSequencer handles INIT → WARMUP → RELOCALIZING → TRACKING
  if (!init_sequencer_.step(timestamp)) {
    publishStatus();
    return;
  }

  // TRACKING — poll plugins, optimize if factors available
  handleTracking(timestamp);

  publishStatus();
}

// ==========================================================================
// Tracking Transition
// ==========================================================================

void EidosNode::beginTracking(const gtsam::Pose3 & initial_pose)
{
  RCLCPP_INFO(
    get_logger(),
    "\033[32m[TRACKING]\033[0m Beginning at pos=(%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f)°",
    initial_pose.translation().x(),
    initial_pose.translation().y(),
    initial_pose.translation().z(),
    initial_pose.rotation().roll() * 180.0 / M_PI,
    initial_pose.rotation().pitch() * 180.0 / M_PI,
    initial_pose.rotation().yaw() * 180.0 / M_PI);

  graph_optimizer_.reset();
  state_.store(SlamState::TRACKING, std::memory_order_release);

  for (auto & plugin : registry_.factor_plugins) {
    plugin->onTrackingBegin(initial_pose);
  }
}

void EidosNode::handleTracking(double timestamp)
{
  // Poll plugins for new factors via produceFactor().
  //  - result.timestamp set → new state created at that timestamp.
  //  - result.timestamp nullopt + factors → standalone factors between existing keys
  //    (e.g. loop closure BetweenFactor).
  std::vector<NewState> new_states;

  gtsam::NonlinearFactorGraph new_factors;
  std::vector<std::string> new_factor_owners;  // parallel to new_factors
  gtsam::Values new_values;
  bool loop_closure_detected = false;
  bool correction_detected = false;

  // Each plugin is called with the next available key. If it returns factors
  // with a timestamp, a new state is created. Factors without a timestamp are
  // added directly (standalone factors between existing keys).
  for (auto & plugin : registry_.factor_plugins) {
    gtsam::Key next_key = gtsam::Symbol('x', graph_optimizer_.getNextStateIndex());
    auto result = plugin->produceFactor(next_key, timestamp);
    if (result.factors.empty() && !result.timestamp.has_value()) continue;

    if (result.timestamp.has_value()) {
      // State-creating plugin (e.g. LISO)
      auto key = graph_optimizer_.createState(result.timestamp.value(), plugin->getName());
      new_states.push_back({key, result.timestamp.value(), std::move(result), plugin->getName()});
    } else {
      // Standalone factors between existing keys (e.g. loop closure).
      for (auto & f : result.factors) {
        new_factors.add(f);
        new_factor_owners.push_back(plugin->getName());
      }
      for (const auto & kv : result.values) {
        if (!new_values.exists(kv.key)) new_values.insert(kv.key, kv.value);
      }
      if (result.loop_closure) loop_closure_detected = true;
      if (result.correction) correction_detected = true;
    }
  }

  // Nothing to optimize this tick
  if (new_states.empty() && new_factors.empty()) {
    publishPose();
    publishOdometry();
    return;
  }

  // Sort new states by timestamp, assign initial values, add prior, and latch plugins
  std::sort(new_states.begin(), new_states.end(), [](auto & a, auto & b) { return a.ts < b.ts; });

  auto current_pose = graph_optimizer_.getOptimizedPose().load().value_or(gtsam::Pose3::Identity());

  for (auto & ns : new_states) {
    // Initial value for this state
    if (ns.result.values.exists(ns.key)) {
      new_values.insert(ns.key, ns.result.values.at<gtsam::Pose3>(ns.key));
    } else {
      new_values.insert(ns.key, current_pose);
    }

    // Factors from the state creator
    for (auto & f : ns.result.factors) {
      new_factors.add(f);
      new_factor_owners.push_back(ns.owner);
    }

    // Additional values (bias keys, etc.)
    for (const auto & kv : ns.result.values) {
      if (!new_values.exists(kv.key)) new_values.insert(kv.key, kv.value);
    }

    // First state ever: add PriorFactor
    if (graph_optimizer_.getNextStateIndex() == 1) {
      gtsam::Pose3 anchor = ns.result.values.exists(ns.key) ? ns.result.values.at<gtsam::Pose3>(ns.key) : current_pose;
      auto & cov = graph_optimizer_.getPriorPoseCov();
      if (cov.size() >= 6) {
        auto noise = gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << cov[3], cov[4], cov[5], cov[0], cov[1], cov[2]).finished());
        new_factors.addPrior(ns.key, anchor, noise);
        new_factor_owners.push_back("prior");
      }
    }

    // Let all plugins latch onto this new state (e.g. GPS, IMU, MotionModel).
    for (auto & plugin : registry_.factor_plugins) {
      auto latch = plugin->latchFactor(ns.key, ns.ts);
      if (latch.factors.empty()) continue;

      for (auto & f : latch.factors) {
        new_factors.add(f);
        new_factor_owners.push_back(plugin->getName());
      }

      // Check if a latching factor was a correction or a loop closure
      if (latch.loop_closure) loop_closure_detected = true;
      if (latch.correction) correction_detected = true;
      for (const auto & kv : latch.values) {
        if (!new_values.exists(kv.key)) new_values.insert(kv.key, kv.value);
      }
    }
  }

  // Update graph adjacency
  map_manager_.addEdges(new_factors, new_factor_owners);

  // Optimize. Use the latest new state key if available, otherwise the last known key.
  gtsam::Key latest_key = new_states.empty() ? last_state_key_ : new_states.back().key;
  int extra = loop_closure_detected ? loop_closure_iterations_ : correction_detected ? correction_iterations_ : 0;
  auto optimized = graph_optimizer_.optimize(new_factors, new_values, latest_key, update_iterations_, extra);

  // Update existing keyframe poses from optimized values, store new keyframes
  map_manager_.updatePoses(optimized);
  for (auto & ns : new_states) {
    gtsam::Pose3 pose = optimized.exists(ns.key) ? optimized.at<gtsam::Pose3>(ns.key) : gtsam::Pose3::Identity();
    PoseType pose_6d = gtsamPose3ToPoseType(pose, ns.ts);
    pose_6d.intensity = static_cast<float>(map_manager_.numKeyframes());
    map_manager_.addKeyframe(ns.key, pose_6d, ns.owner);
  }

  // Notify plugins of optimization result (re-anchor GICP guess, rebuild submap, etc.)
  for (auto & plugin : registry_.factor_plugins) {
    plugin->onOptimizationComplete(optimized, loop_closure_detected || correction_detected);
  }

  // Track latest state for cross-plugin bridging
  auto & last = new_states.back();
  last_state_key_ = last.key;
  last_state_ts_ = last.ts;
  last_state_owner_ = last.owner;
  has_last_state_ = true;

  RCLCPP_INFO(
    get_logger(),
    "\033[32m[TRACKING]\033[0m states=%zu factors=%zu pos=(%.3f, %.3f, %.3f)",
    new_states.size(),
    new_factors.size(),
    optimized.at<gtsam::Pose3>(latest_key).translation().x(),
    optimized.at<gtsam::Pose3>(latest_key).translation().y(),
    optimized.at<gtsam::Pose3>(latest_key).translation().z());

  publishPose();
  publishOdometry();
}

// ==========================================================================
// Publishing
// ==========================================================================

void EidosNode::publishStatus()
{
  if (!status_pub_->is_activated()) return;

  eidos_msgs::msg::SlamStatus msg;
  msg.header.stamp = now();
  msg.state = static_cast<uint8_t>(state_.load(std::memory_order_acquire));
  msg.current_state_index = static_cast<int>(graph_optimizer_.getNextStateIndex());
  msg.num_keyframes = map_manager_.numKeyframes();
  msg.num_factors = graph_optimizer_.numFactors();

  for (auto & p : registry_.factor_plugins) msg.active_factor_plugins.push_back(p->getName());
  for (auto & p : registry_.reloc_plugins) msg.active_relocalization_plugins.push_back(p->getName());

  status_pub_->publish(msg);
}

std::optional<gtsam::Pose3> EidosNode::getCurrentPose() const
{
  auto pose = graph_optimizer_.getOptimizedPose().load();
  if (pose) return pose;
  for (auto & p : registry_.factor_plugins) {
    auto map_pose = p->getMapPose();
    if (map_pose) return map_pose;
  }
  return std::nullopt;
}

void EidosNode::publishPose()
{
  if (!pose_pub_->is_activated()) return;

  auto pose = getCurrentPose();
  if (!pose) return;

  geometry_msgs::msg::PoseStamped msg;
  // Stamp with the keyframe sensor time so eidos_transform can look up
  // the matching odom_to_base for a precise map->odom correction.
  msg.header.stamp = has_last_state_ ? rclcpp::Time(static_cast<int64_t>(last_state_ts_ * 1e9), RCL_ROS_TIME) : now();
  msg.header.frame_id = map_frame_;

  auto q = pose->rotation().toQuaternion();
  msg.pose.position.x = pose->translation().x();
  msg.pose.position.y = pose->translation().y();
  msg.pose.position.z = pose->translation().z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  pose_pub_->publish(msg);
}

void EidosNode::publishOdometry()
{
  if (!odom_pub_->is_activated()) return;

  auto pose = getCurrentPose();
  if (!pose) return;

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = has_last_state_ ? rclcpp::Time(static_cast<int64_t>(last_state_ts_ * 1e9), RCL_ROS_TIME) : now();
  msg.header.frame_id = map_frame_;
  msg.child_frame_id = base_link_frame_;

  auto q = pose->rotation().toQuaternion();
  msg.pose.pose.position.x = pose->translation().x();
  msg.pose.pose.position.y = pose->translation().y();
  msg.pose.pose.position.z = pose->translation().z();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  for (size_t i = 0; i < odom_pose_cov_.size() && i < 6; i++) {
    msg.pose.covariance[i * 7] = odom_pose_cov_[i];
  }

  odom_pub_->publish(msg);
}

// ==========================================================================
// Services
// ==========================================================================

void EidosNode::saveMapCallback(
  const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response)
{
  std::string path = request->filepath.empty() ? map_save_directory_ : request->filepath;
  RCLCPP_INFO(get_logger(), "\033[36m[SAVE]\033[0m Saving map to: %s", path.c_str());

  bool ok = map_manager_.saveMap(path);
  response->success = ok;
  response->message = ok ? "Map saved to " + path : "Failed to save map";
}

void EidosNode::loadMapCallback(
  const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response)
{
  RCLCPP_INFO(get_logger(), "\033[36m[LOAD]\033[0m Loading map from: %s", request->filepath.c_str());

  bool ok = map_manager_.loadMap(request->filepath);
  response->success = ok;
  response->message = ok ? "Map loaded" : "Failed to load map";
}

}  // namespace eidos
