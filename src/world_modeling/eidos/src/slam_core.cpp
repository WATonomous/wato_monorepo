#include "eidos/slam_core.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include <gtsam/slam/BetweenFactor.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace eidos {

SlamCore::SlamCore(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("eidos_node", options) {
  RCLCPP_INFO(get_logger(), "EidosNode created (unconfigured)");
}

SlamCore::~SlamCore() = default;

// ---- Lifecycle callbacks ----

SlamCore::CallbackReturn SlamCore::on_configure(
    const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring...");

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
      shared_from_this());

  // Parameters
  declare_parameter("slam_rate", slam_rate_);
  declare_parameter("max_displacement", max_displacement_);
  declare_parameter("max_rotation", max_rotation_);
  declare_parameter("relocalization_timeout", relocalization_timeout_);
  declare_parameter("frames.map", map_frame_);
  declare_parameter("frames.odometry", odom_frame_);
  declare_parameter("frames.base_link", base_link_frame_);
  declare_parameter("keyframe.density", keyframe_density_);
  declare_parameter("keyframe.search_radius", keyframe_search_radius_);
  declare_parameter("map.load_directory", map_load_directory_);
  declare_parameter("map.save_directory", map_save_directory_);
  declare_parameter("factor_plugins", std::vector<std::string>{});
  declare_parameter("relocalization_plugins", std::vector<std::string>{});
  declare_parameter("visualization_plugins", std::vector<std::string>{});
  declare_parameter("topics.path", "slam/path");
  declare_parameter("topics.status", "slam/status");
  declare_parameter("topics.save_map_service", "slam/save_map");
  declare_parameter("topics.load_map_service", "slam/load_map");

  get_parameter("slam_rate", slam_rate_);
  get_parameter("max_displacement", max_displacement_);
  get_parameter("max_rotation", max_rotation_);
  get_parameter("relocalization_timeout", relocalization_timeout_);
  get_parameter("frames.map", map_frame_);
  get_parameter("frames.odometry", odom_frame_);
  get_parameter("frames.base_link", base_link_frame_);
  get_parameter("keyframe.density", keyframe_density_);
  get_parameter("keyframe.search_radius", keyframe_search_radius_);
  get_parameter("map.load_directory", map_load_directory_);
  get_parameter("map.save_directory", map_save_directory_);

  std::string path_topic, status_topic, save_map_service, load_map_service;
  get_parameter("topics.path", path_topic);
  get_parameter("topics.status", status_topic);
  get_parameter("topics.save_map_service", save_map_service);
  get_parameter("topics.load_map_service", load_map_service);

  // Core components
  pose_graph_ = std::make_unique<PoseGraph>();
  map_manager_ = std::make_unique<MapManager>();

  // Publishers
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 1);
  status_pub_ = create_publisher<eidos_msgs::msg::SlamStatus>(status_topic, 1);

  // Services
  save_map_srv_ = create_service<eidos_msgs::srv::SaveMap>(
      save_map_service,
      std::bind(&SlamCore::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = create_service<eidos_msgs::srv::LoadMap>(
      load_map_service,
      std::bind(&SlamCore::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Load plugins
  factor_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<FactorPlugin>>(
      "eidos", "eidos::FactorPlugin");
  reloc_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<RelocalizationPlugin>>(
      "eidos", "eidos::RelocalizationPlugin");
  vis_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<VisualizationPlugin>>(
      "eidos", "eidos::VisualizationPlugin");

  loadFactorPlugins();
  loadRelocalizationPlugins();
  loadVisualizationPlugins();

  RCLCPP_INFO(get_logger(), "\033[36m[CONFIGURED]\033[0m Eidos SLAM configured");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_activate(
    const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATING]\033[0m ...");

  // Load prior map if configured
  if (!map_load_directory_.empty()) {
    if (map_manager_->loadMap(map_load_directory_)) {
      RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATING]\033[0m Loaded prior map from: %s",
                  map_load_directory_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "\033[36m[ACTIVATING]\033[0m Failed to load prior map from: %s",
                  map_load_directory_.c_str());
    }
  }

  // Always enter WARMING_UP — sensors must report ready before SLAM starts
  state_ = SlamState::WARMING_UP;
  RCLCPP_INFO(get_logger(), "\033[33m[WARMING_UP]\033[0m Waiting for factors...");

  // Activate lifecycle publishers
  path_pub_->on_activate();
  status_pub_->on_activate();

  // Activate plugins
  for (auto& plugin : factor_plugins_) {
    plugin->activate();
  }
  for (auto& plugin : reloc_plugins_) {
    plugin->activate();
  }
  for (auto& plugin : vis_plugins_) {
    plugin->activate();
  }

  // Start SLAM timer
  slam_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto period = std::chrono::duration<double>(1.0 / slam_rate_);
  slam_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlamCore::slamLoop, this),
      slam_callback_group_);

  RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATED]\033[0m Eidos SLAM active");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_deactivate(
    const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "\033[36m[DEACTIVATING]\033[0m ...");

  if (slam_timer_) {
    slam_timer_->cancel();
    slam_timer_.reset();
  }

  // Deactivate lifecycle publishers
  path_pub_->on_deactivate();
  status_pub_->on_deactivate();

  for (auto& plugin : factor_plugins_) {
    plugin->deactivate();
  }
  for (auto& plugin : reloc_plugins_) {
    plugin->deactivate();
  }
  for (auto& plugin : vis_plugins_) {
    plugin->deactivate();
  }

  RCLCPP_INFO(get_logger(), "\033[36m[DEACTIVATED]\033[0m Eidos SLAM deactivated");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_cleanup(
    const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "\033[36m[CLEANING_UP]\033[0m ...");

  // Release plugins before loaders
  factor_plugins_.clear();
  reloc_plugins_.clear();
  vis_plugins_.clear();
  factor_plugin_loader_.reset();
  reloc_plugin_loader_.reset();
  vis_plugin_loader_.reset();

  path_pub_.reset();
  status_pub_.reset();
  save_map_srv_.reset();
  load_map_srv_.reset();

  pose_graph_.reset();
  map_manager_.reset();

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // Reset SLAM state
  state_ = SlamState::INITIALIZING;
  current_pose_ = gtsam::Pose3();
  current_state_index_ = -1;
  std::fill(std::begin(current_transform_), std::end(current_transform_), 0.0f);
  loop_closure_detected_ = false;
  accumulated_graph_ = gtsam::NonlinearFactorGraph();
  accumulated_values_ = gtsam::Values();
  global_path_ = nav_msgs::msg::Path();

  RCLCPP_INFO(get_logger(), "\033[36m[CLEANED_UP]\033[0m Eidos SLAM cleaned up");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "\033[36m[SHUTTING_DOWN]\033[0m ...");

  // If still active, deactivate first
  if (slam_timer_) {
    on_deactivate(state);
  }
  on_cleanup(state);

  RCLCPP_INFO(get_logger(), "\033[36m[SHUT_DOWN]\033[0m Eidos SLAM shut down");
  return CallbackReturn::SUCCESS;
}

// ---- SLAM loop ----

void SlamCore::slamLoop() {
  bool run_vis = false;
  gtsam::Values vis_values;
  bool vis_loop_closure = false;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    double timestamp = now().seconds();

    switch (state_) {
      case SlamState::INITIALIZING:
        handleInitializing();
        break;
      case SlamState::WARMING_UP:
        handleWarmingUp(timestamp);
        break;
      case SlamState::RELOCALIZING:
        handleRelocalizing(timestamp);
        break;
      case SlamState::TRACKING:
        handleTracking(timestamp, run_vis, vis_values, vis_loop_closure);
        break;
    }

    publishStatus();
  }

  // Visualization and path publishing run outside the lock — they are
  // read-only with respect to SLAM state and can tolerate slightly stale data.
  if (run_vis) {
    for (auto& plugin : vis_plugins_) {
      plugin->onOptimizationComplete(vis_values, vis_loop_closure);
    }
  }
  publishPath();
}

void SlamCore::handleInitializing() {
  // Always transition to WARMING_UP
  state_ = SlamState::WARMING_UP;
  RCLCPP_INFO(get_logger(), "\033[33m[WARMING_UP]\033[0m Transitioning from INITIALIZING");
}

void SlamCore::handleWarmingUp(double timestamp) {
  int ready_count = 0;
  int total = static_cast<int>(factor_plugins_.size());
  std::string report;

  for (auto& plugin : factor_plugins_) {
    bool ready = plugin->isReady();
    if (ready) ready_count++;
    std::string status = plugin->getReadyStatus();
    report += "\n  " + plugin->getName() + ": "
           + (ready ? "\033[32mREADY\033[0m" : "\033[33mWAITING\033[0m");
    if (!status.empty()) report += " — " + status;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "\033[33m[WARMING_UP]\033[0m factors ready: %d/%d%s",
      ready_count, total, report.c_str());

  if (ready_count < total) return;

  RCLCPP_INFO(get_logger(),
      "\033[32m[READY]\033[0m All factor plugins ready");

  if (!reloc_plugins_.empty() && map_manager_->hasPriorMap()) {
    state_ = SlamState::RELOCALIZING;
    relocalization_start_time_ = timestamp;
    RCLCPP_INFO(get_logger(), "\033[35m[RELOCALIZING]\033[0m Prior map loaded, entering relocalization");
  } else {
    beginTracking(gtsam::Pose3::Identity(), timestamp);
  }
}

void SlamCore::beginTracking(const gtsam::Pose3& initial_pose, double timestamp) {
  RCLCPP_INFO(get_logger(), "\033[32m[TRACKING]\033[0m Beginning at [%.2f, %.2f, %.2f]",
      initial_pose.translation().x(), initial_pose.translation().y(),
      initial_pose.translation().z());

  current_pose_ = initial_pose;
  current_transform_[0] = static_cast<float>(initial_pose.rotation().roll());
  current_transform_[1] = static_cast<float>(initial_pose.rotation().pitch());
  current_transform_[2] = static_cast<float>(initial_pose.rotation().yaw());
  current_transform_[3] = static_cast<float>(initial_pose.translation().x());
  current_transform_[4] = static_cast<float>(initial_pose.translation().y());
  current_transform_[5] = static_cast<float>(initial_pose.translation().z());
  current_state_index_ = 0;

  // Create first keyframe with prior factor
  gtsam::NonlinearFactorGraph initial_factors;
  gtsam::Values initial_values;
  initial_values.insert(0, current_pose_);
  auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
  initial_factors.addPrior(0, current_pose_, prior_noise);

  // Collect factors from all plugins for state 0
  for (auto& plugin : factor_plugins_) {
    auto factors = plugin->getFactors(0, current_pose_, timestamp);
    for (auto& f : factors) initial_factors.add(f);
  }

  // Store and optimize
  accumulated_graph_.add(initial_factors);
  if (!accumulated_values_.exists(0)) {
    accumulated_values_.insert(0, current_pose_);
  }
  auto optimized = pose_graph_->update(initial_factors, initial_values);

  // Store keyframe
  PoseType pose_6d = gtsamPose3ToPoseType(current_pose_, timestamp);
  pose_6d.intensity = 0.0f;
  map_manager_->addKeyframe(0, pose_6d);

  // Set state before notifying plugins so their callbacks can start buffering
  state_ = SlamState::TRACKING;

  // Notify all factors of initial optimization
  for (auto& plugin : factor_plugins_) {
    plugin->onOptimizationComplete(optimized, false);
  }

  // Notify visualizers with initial state
  for (auto& plugin : vis_plugins_) {
    plugin->onOptimizationComplete(optimized, false);
  }

  updatePath(pose_6d);
  broadcastMapToOdom();
}

void SlamCore::handleRelocalizing(double timestamp) {
  // Keep factor plugins warmed up (buffering data)
  for (auto& plugin : factor_plugins_) {
    plugin->processFrame(timestamp);
  }

  // Try each relocalization plugin
  for (auto& plugin : reloc_plugins_) {
    auto result = plugin->tryRelocalize(timestamp);
    if (result.has_value()) {
      RCLCPP_INFO(get_logger(),
                  "\033[35m[RELOCALIZING]\033[0m Succeeded (fitness: %.3f, keyframe: %d)",
                  result->fitness_score, result->matched_keyframe_index);
      beginTracking(result->pose, timestamp);
      return;
    }
  }

  // Timeout fallback
  double elapsed = timestamp - relocalization_start_time_;
  if (elapsed > relocalization_timeout_) {
    RCLCPP_WARN(get_logger(),
                "\033[35m[RELOCALIZING]\033[0m Timed out after %.1f seconds, starting from origin",
                elapsed);
    beginTracking(gtsam::Pose3::Identity(), timestamp);
  }
}

void SlamCore::handleTracking(double timestamp, bool& run_vis,
                              gtsam::Values& vis_values,
                              bool& vis_loop_closure) {
  // 1. Process frames from all factor plugins
  std::optional<gtsam::Pose3> best_pose;
  for (auto& plugin : factor_plugins_) {
    auto pose = plugin->processFrame(timestamp);
    if (pose.has_value() && !best_pose.has_value()) {
      best_pose = pose;  // First valid pose wins (plugin order = priority)
    }
  }

  if (!best_pose.has_value()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "\033[32m[TRACKING]\033[0m Waiting for first pose from factor plugins...");
    return;  // No pose estimate yet
  }

  current_pose_ = best_pose.value();
  current_transform_[0] = static_cast<float>(current_pose_.rotation().roll());
  current_transform_[1] = static_cast<float>(current_pose_.rotation().pitch());
  current_transform_[2] = static_cast<float>(current_pose_.rotation().yaw());
  current_transform_[3] = static_cast<float>(current_pose_.translation().x());
  current_transform_[4] = static_cast<float>(current_pose_.translation().y());
  current_transform_[5] = static_cast<float>(current_pose_.translation().z());

  // 2. Check displacement since last state (state 0 is handled by beginTracking)
  bool new_state = false;
  auto poses_6d = map_manager_->getKeyPoses6D();
  if (poses_6d->empty()) {
    return;  // beginTracking() not yet called — should not happen
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

    // Signal visualization to run outside the lock
    run_vis = true;
    vis_values = optimized;
    vis_loop_closure = loop_closure_detected_;

    // Broadcast map → odom TF
    broadcastMapToOdom();

    // Update path
    updatePath(pose_6d);
  }
}

// ---- Plugin loading ----

void SlamCore::loadFactorPlugins() {
  std::vector<std::string> plugin_names;
  get_parameter("factor_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    declare_parameter(name + ".plugin", std::string{});
    get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Factor plugin '%s' has no 'plugin' parameter.", name.c_str());
      continue;
    }

    try {
      auto plugin = factor_plugin_loader_->createSharedInstance(plugin_type);
      auto cb_group = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      plugin->initialize(this, name, shared_from_this(), tf_buffer_.get(), cb_group);
      factor_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded factor plugin: %s (%s)",
                  name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load factor plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

void SlamCore::loadRelocalizationPlugins() {
  std::vector<std::string> plugin_names;
  get_parameter("relocalization_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    declare_parameter(name + ".plugin", std::string{});
    get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Relocalization plugin '%s' has no 'plugin' parameter.",
                   name.c_str());
      continue;
    }

    try {
      auto plugin = reloc_plugin_loader_->createSharedInstance(plugin_type);
      auto cb_group = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      plugin->initialize(this, name, shared_from_this(), tf_buffer_.get(), cb_group);
      reloc_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded relocalization plugin: %s (%s)",
                  name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load relocalization plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

void SlamCore::loadVisualizationPlugins() {
  std::vector<std::string> plugin_names;
  get_parameter("visualization_plugins", plugin_names);

  for (const auto& name : plugin_names) {
    std::string plugin_type;
    declare_parameter(name + ".plugin", std::string{});
    get_parameter(name + ".plugin", plugin_type);

    if (plugin_type.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Visualization plugin '%s' has no 'plugin' parameter.",
                   name.c_str());
      continue;
    }

    try {
      auto plugin = vis_plugin_loader_->createSharedInstance(plugin_type);
      auto cb_group = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      plugin->initialize(this, name, shared_from_this(), tf_buffer_.get(), cb_group);
      vis_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded visualization plugin: %s (%s)",
                  name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load visualization plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

// ---- Services ----

void SlamCore::saveMapCallback(
    const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response) {
  std::lock_guard<std::mutex> lock(mtx_);
  std::string dir = request->directory.empty() ? map_save_directory_ : request->directory;

  RCLCPP_INFO(get_logger(), "Saving map to: %s", dir.c_str());
  bool success = map_manager_->saveMap(
      dir, request->resolution, accumulated_graph_, accumulated_values_);

  response->success = success;
  response->message = success ? "Map saved successfully" : "Failed to save map";
}

void SlamCore::loadMapCallback(
    const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response) {
  std::lock_guard<std::mutex> lock(mtx_);

  RCLCPP_INFO(get_logger(), "Loading map from: %s", request->directory.c_str());
  bool success = map_manager_->loadMap(request->directory);

  response->success = success;
  response->num_keyframes = map_manager_->numKeyframes();
  response->message = success ? "Map loaded successfully" : "Failed to load map";
}

// ---- Publishing ----

void SlamCore::publishStatus() {
  auto msg = eidos_msgs::msg::SlamStatus();
  msg.header.stamp = now();
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

  global_path_.header.stamp = now();
  global_path_.header.frame_id = map_frame_;
  path_pub_->publish(global_path_);
}

void SlamCore::updatePath(const PoseType& pose) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = rclcpp::Time(static_cast<int64_t>(pose.time * 1e9));
  ps.header.frame_id = map_frame_;
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

void SlamCore::broadcastMapToOdom() {
  // T_map_odom = T_map_base * inverse(T_odom_base)
  // T_map_base = latest optimized pose from ISAM2
  // T_odom_base = looked up from TF tree (published by IMU factor)

  // Look up T_odom_base from TF tree
  geometry_msgs::msg::TransformStamped t_odom_base_msg;
  try {
    t_odom_base_msg = tf_buffer_->lookupTransform(
        odom_frame_, base_link_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException&) {
    return;  // TF not yet available (IMU factor hasn't published)
  }

  // Build T_map_base from current optimized pose
  tf2::Transform tf_map_base;
  tf2::Quaternion q_map_base;
  q_map_base.setRPY(current_pose_.rotation().roll(),
                     current_pose_.rotation().pitch(),
                     current_pose_.rotation().yaw());
  tf_map_base.setRotation(q_map_base);
  tf_map_base.setOrigin({current_pose_.translation().x(),
                          current_pose_.translation().y(),
                          current_pose_.translation().z()});

  // Convert T_odom_base msg → tf2::Transform
  tf2::Transform tf_odom_base;
  tf2::fromMsg(t_odom_base_msg.transform, tf_odom_base);

  // Compose: T_map_odom = T_map_base * inv(T_odom_base)
  tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

  // Convert back to msg and broadcast
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;
  tf_msg.transform = tf2::toMsg(tf_map_odom);

  tf_broadcaster_->sendTransform(tf_msg);
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
