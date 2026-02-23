#include "eidos/slam_core.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include <gtsam/slam/BetweenFactor.h>

namespace eidos {

SlamCore::SlamCore(rclcpp::Node::SharedPtr node)
    : node_(node) {
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  node_->declare_parameter("slam_rate", slam_rate_);
  node_->declare_parameter("max_displacement", max_displacement_);
  node_->declare_parameter("max_rotation", max_rotation_);
  node_->declare_parameter("relocalization_timeout", relocalization_timeout_);
  node_->declare_parameter("frames.map", map_frame_);
  node_->declare_parameter("frames.odometry", odom_frame_);
  node_->declare_parameter("keyframe.density", keyframe_density_);
  node_->declare_parameter("keyframe.search_radius", keyframe_search_radius_);
  node_->declare_parameter("map.load_directory", map_load_directory_);
  node_->declare_parameter("map.save_directory", map_save_directory_);
  node_->declare_parameter("factor_plugins", std::vector<std::string>{});
  node_->declare_parameter("relocalization_plugins", std::vector<std::string>{});

  node_->get_parameter("slam_rate", slam_rate_);
  node_->get_parameter("max_displacement", max_displacement_);
  node_->get_parameter("max_rotation", max_rotation_);
  node_->get_parameter("relocalization_timeout", relocalization_timeout_);
  node_->get_parameter("frames.map", map_frame_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("keyframe.density", keyframe_density_);
  node_->get_parameter("keyframe.search_radius", keyframe_search_radius_);
  node_->get_parameter("map.load_directory", map_load_directory_);
  node_->get_parameter("map.save_directory", map_save_directory_);

  // Core components
  pose_graph_ = std::make_unique<PoseGraph>();
  map_manager_ = std::make_unique<MapManager>();

  // Publishers
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("slam/path", 1);
  status_pub_ = node_->create_publisher<eidos_msgs::msg::SlamStatus>("slam/status", 1);

  // Services
  save_map_srv_ = node_->create_service<eidos_msgs::srv::SaveMap>(
      "slam/save_map",
      std::bind(&SlamCore::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = node_->create_service<eidos_msgs::srv::LoadMap>(
      "slam/load_map",
      std::bind(&SlamCore::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Load plugins
  factor_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<FactorPlugin>>(
      "eidos", "eidos::FactorPlugin");
  reloc_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<RelocalizationPlugin>>(
      "eidos", "eidos::RelocalizationPlugin");

  loadFactorPlugins();
  loadRelocalizationPlugins();
}

SlamCore::~SlamCore() {
  stop();
}

void SlamCore::start() {
  // Load prior map if configured
  if (!map_load_directory_.empty()) {
    if (map_manager_->loadMap(map_load_directory_)) {
      RCLCPP_INFO(node_->get_logger(), "Loaded prior map from: %s",
                  map_load_directory_.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to load prior map from: %s",
                  map_load_directory_.c_str());
    }
  }

  // Determine initial state
  if (!reloc_plugins_.empty() && map_manager_->hasPriorMap()) {
    state_ = SlamState::RELOCALIZING;
    relocalization_start_time_ = node_->now().seconds();
    RCLCPP_INFO(node_->get_logger(), "Entering RELOCALIZING state");
  } else {
    state_ = SlamState::TRACKING;
    RCLCPP_INFO(node_->get_logger(), "Entering TRACKING state (fresh SLAM)");
  }

  // Activate plugins
  for (auto& plugin : factor_plugins_) {
    plugin->activate();
  }
  for (auto& plugin : reloc_plugins_) {
    plugin->activate();
  }

  // Start SLAM timer
  slam_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto period = std::chrono::duration<double>(1.0 / slam_rate_);
  slam_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlamCore::slamLoop, this),
      slam_callback_group_);
}

void SlamCore::stop() {
  if (slam_timer_) {
    slam_timer_->cancel();
    slam_timer_.reset();
  }
  for (auto& plugin : factor_plugins_) {
    plugin->deactivate();
  }
  for (auto& plugin : reloc_plugins_) {
    plugin->deactivate();
  }
}

void SlamCore::slamLoop() {
  std::lock_guard<std::mutex> lock(mtx_);
  double timestamp = node_->now().seconds();

  switch (state_) {
    case SlamState::INITIALIZING:
      handleInitializing();
      break;
    case SlamState::RELOCALIZING:
      handleRelocalizing(timestamp);
      break;
    case SlamState::TRACKING:
      handleTracking(timestamp);
      break;
  }

  publishStatus();
}

void SlamCore::handleInitializing() {
  // Transition to appropriate state
  if (!reloc_plugins_.empty() && map_manager_->hasPriorMap()) {
    state_ = SlamState::RELOCALIZING;
    relocalization_start_time_ = node_->now().seconds();
  } else {
    state_ = SlamState::TRACKING;
  }
}

void SlamCore::handleRelocalizing(double timestamp) {
  // Keep factor plugins warmed up
  for (auto& plugin : factor_plugins_) {
    plugin->processFrame(timestamp);
  }

  // Try each relocalization plugin
  for (auto& plugin : reloc_plugins_) {
    auto result = plugin->tryRelocalize(timestamp);
    if (result.has_value()) {
      current_pose_ = result->pose;

      // Convert to transform array
      current_transform_[0] = static_cast<float>(result->pose.rotation().roll());
      current_transform_[1] = static_cast<float>(result->pose.rotation().pitch());
      current_transform_[2] = static_cast<float>(result->pose.rotation().yaw());
      current_transform_[3] = static_cast<float>(result->pose.translation().x());
      current_transform_[4] = static_cast<float>(result->pose.translation().y());
      current_transform_[5] = static_cast<float>(result->pose.translation().z());

      RCLCPP_INFO(node_->get_logger(),
                  "Relocalization succeeded (fitness: %.3f, keyframe: %d). "
                  "Transitioning to TRACKING.",
                  result->fitness_score, result->matched_keyframe_index);
      state_ = SlamState::TRACKING;
      return;
    }
  }

  // Check timeout
  double elapsed = timestamp - relocalization_start_time_;
  if (elapsed > relocalization_timeout_) {
    RCLCPP_WARN(node_->get_logger(),
                "Relocalization timed out after %.1f seconds. "
                "Falling back to fresh SLAM.",
                elapsed);
    state_ = SlamState::TRACKING;
  }
}

void SlamCore::handleTracking(double timestamp) {
  // 1. Process frames from all factor plugins
  std::optional<gtsam::Pose3> best_pose;
  for (auto& plugin : factor_plugins_) {
    auto pose = plugin->processFrame(timestamp);
    if (pose.has_value() && !best_pose.has_value()) {
      best_pose = pose;  // First valid pose wins (plugin order = priority)
    }
  }

  if (!best_pose.has_value()) {
    return;  // No pose estimate yet
  }

  current_pose_ = best_pose.value();
  current_transform_[0] = static_cast<float>(current_pose_.rotation().roll());
  current_transform_[1] = static_cast<float>(current_pose_.rotation().pitch());
  current_transform_[2] = static_cast<float>(current_pose_.rotation().yaw());
  current_transform_[3] = static_cast<float>(current_pose_.translation().x());
  current_transform_[4] = static_cast<float>(current_pose_.translation().y());
  current_transform_[5] = static_cast<float>(current_pose_.translation().z());

  // 2. Check displacement since last state
  bool new_state = false;
  auto poses_6d = map_manager_->getKeyPoses6D();
  if (current_state_index_ < 0 || poses_6d->empty()) {
    new_state = true;  // First state
  } else {
    auto last_pose = poses_6d->points.back();
    Eigen::Affine3f last_affine = poseTypeToAffine3f(last_pose);
    Eigen::Affine3f current_affine = rpyxyzToAffine3f(current_transform_);
    Eigen::Affine3f delta = last_affine.inverse() * current_affine;

    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(delta, dx, dy, dz, droll, dpitch, dyaw);

    float translation = std::sqrt(dx * dx + dy * dy + dz * dz);
    float rotation = std::max({std::abs(droll), std::abs(dpitch), std::abs(dyaw)});

    if (translation > max_displacement_ || rotation > max_rotation_) {
      new_state = true;
    }
  }

  // 3. If new state, collect factors and optimize
  if (new_state) {
    current_state_index_++;

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    new_values.insert(current_state_index_, current_pose_);

    // Add prior factor for first state
    if (current_state_index_ == 0) {
      auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
      new_factors.addPrior(0, current_pose_, prior_noise);
    }

    // Collect factors from all plugins
    loop_closure_detected_ = false;
    for (auto& plugin : factor_plugins_) {
      auto factors = plugin->getFactors(
          current_state_index_, current_pose_, timestamp);
      for (auto& f : factors) {
        new_factors.add(f);
        // Detect loop closure: BetweenFactors connecting non-consecutive states
        auto between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f);
        if (between) {
          auto keys = between->keys();
          if (keys.size() == 2) {
            int diff = std::abs(static_cast<int>(keys[0]) -
                                static_cast<int>(keys[1]));
            if (diff > 1) {
              loop_closure_detected_ = true;
            }
          }
        }
      }
    }

    // Track accumulated graph for serialization
    accumulated_graph_.add(new_factors);
    if (!accumulated_values_.exists(current_state_index_)) {
      accumulated_values_.insert(current_state_index_, current_pose_);
    }

    // Optimize
    auto optimized = pose_graph_->update(new_factors, new_values);

    // Check if any loop closure factors were added
    // (plugins signal this by adding BetweenFactors that aren't sequential)
    // Update poses if loop closure detected
    if (loop_closure_detected_) {
      pose_graph_->updateExtra(5);
      optimized = pose_graph_->getOptimizedValues();
      map_manager_->updatePoses(optimized);

      // Rebuild path
      global_path_.poses.clear();
      for (int i = 0; i <= current_state_index_; i++) {
        auto pose = optimized.at<gtsam::Pose3>(i);
        PoseType pt = gtsamPose3ToPoseType(pose);
        updatePath(pt);
      }
    }

    // Get latest optimized pose
    auto latest_pose = optimized.at<gtsam::Pose3>(current_state_index_);
    current_pose_ = latest_pose;
    current_transform_[0] = static_cast<float>(latest_pose.rotation().roll());
    current_transform_[1] = static_cast<float>(latest_pose.rotation().pitch());
    current_transform_[2] = static_cast<float>(latest_pose.rotation().yaw());
    current_transform_[3] = static_cast<float>(latest_pose.translation().x());
    current_transform_[4] = static_cast<float>(latest_pose.translation().y());
    current_transform_[5] = static_cast<float>(latest_pose.translation().z());

    // Store keyframe pose (plugins store their own data via addKeyframeData)
    PoseType pose_6d = gtsamPose3ToPoseType(latest_pose, timestamp);
    pose_6d.intensity = static_cast<float>(current_state_index_);
    map_manager_->addKeyframe(current_state_index_, pose_6d);

    // Notify plugins of optimization result
    for (auto& plugin : factor_plugins_) {
      plugin->onOptimizationComplete(optimized, loop_closure_detected_);
    }

    // Update path
    updatePath(pose_6d);
  }

  publishPath();
}

void SlamCore::loadFactorPlugins() {
  std::vector<std::string> plugin_names;
  node_->get_parameter("factor_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    node_->declare_parameter(name + ".plugin", std::string{});
    node_->get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Factor plugin '%s' has no 'plugin' parameter.", name.c_str());
      continue;
    }

    try {
      auto plugin = factor_plugin_loader_->createSharedInstance(plugin_type);
      auto cb_group = node_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      plugin->initialize(this, name, node_, tf_buffer_.get(), cb_group);
      factor_plugins_.push_back(plugin);
      RCLCPP_INFO(node_->get_logger(), "Loaded factor plugin: %s (%s)",
                  name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to load factor plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

void SlamCore::loadRelocalizationPlugins() {
  std::vector<std::string> plugin_names;
  node_->get_parameter("relocalization_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    node_->declare_parameter(name + ".plugin", std::string{});
    node_->get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Relocalization plugin '%s' has no 'plugin' parameter.",
                   name.c_str());
      continue;
    }

    try {
      auto plugin = reloc_plugin_loader_->createSharedInstance(plugin_type);
      auto cb_group = node_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      plugin->initialize(this, name, node_, tf_buffer_.get(), cb_group);
      reloc_plugins_.push_back(plugin);
      RCLCPP_INFO(node_->get_logger(), "Loaded relocalization plugin: %s (%s)",
                  name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to load relocalization plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

void SlamCore::saveMapCallback(
    const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response) {
  std::lock_guard<std::mutex> lock(mtx_);
  std::string dir = request->directory.empty() ? map_save_directory_ : request->directory;

  RCLCPP_INFO(node_->get_logger(), "Saving map to: %s", dir.c_str());
  bool success = map_manager_->saveMap(
      dir, request->resolution, accumulated_graph_, accumulated_values_);

  response->success = success;
  response->message = success ? "Map saved successfully" : "Failed to save map";
}

void SlamCore::loadMapCallback(
    const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response) {
  std::lock_guard<std::mutex> lock(mtx_);

  RCLCPP_INFO(node_->get_logger(), "Loading map from: %s", request->directory.c_str());
  bool success = map_manager_->loadMap(request->directory);

  response->success = success;
  response->num_keyframes = map_manager_->numKeyframes();
  response->message = success ? "Map loaded successfully" : "Failed to load map";
}

void SlamCore::publishStatus() {
  auto msg = eidos_msgs::msg::SlamStatus();
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_;
  msg.state = static_cast<uint8_t>(state_);
  msg.current_state_index = current_state_index_;
  msg.num_keyframes = map_manager_->numKeyframes();
  msg.num_factors = pose_graph_->numFactors();
  msg.loop_closure_detected = loop_closure_detected_;

  for (const auto& plugin : factor_plugins_) {
    msg.active_factor_plugins.push_back(plugin->getName());
  }
  for (const auto& plugin : reloc_plugins_) {
    msg.active_relocalization_plugins.push_back(plugin->getName());
  }

  status_pub_->publish(msg);
}

void SlamCore::publishPath() {
  if (path_pub_->get_subscription_count() == 0) return;

  global_path_.header.stamp = node_->now();
  global_path_.header.frame_id = odom_frame_;
  path_pub_->publish(global_path_);
}

void SlamCore::updatePath(const PoseType& pose) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = rclcpp::Time(static_cast<int64_t>(pose.time * 1e9));
  ps.header.frame_id = odom_frame_;
  ps.pose.position.x = pose.x;
  ps.pose.position.y = pose.y;
  ps.pose.position.z = pose.z;

  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  global_path_.poses.push_back(ps);
}

// ---- Read-only accessors for plugins ----

const PoseGraph& SlamCore::getPoseGraph() const {
  return *pose_graph_;
}

const MapManager& SlamCore::getMapManager() const {
  return *map_manager_;
}

MapManager& SlamCore::getMapManager() {
  return *map_manager_;
}

SlamState SlamCore::getState() const {
  return state_;
}

gtsam::Pose3 SlamCore::getCurrentPose() const {
  return current_pose_;
}

int SlamCore::getCurrentStateIndex() const {
  return current_state_index_;
}

}  // namespace eidos
