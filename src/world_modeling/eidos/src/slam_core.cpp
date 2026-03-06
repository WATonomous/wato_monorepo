#include "eidos/slam_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include <Eigen/Geometry>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
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
  declare_parameter("prior.pose_cov", prior_pose_cov_);
  declare_parameter("factor_plugins", std::vector<std::string>{});
  declare_parameter("motion_model.plugin", std::string{});
  declare_parameter("relocalization_plugins", std::vector<std::string>{});
  declare_parameter("visualization_plugins", std::vector<std::string>{});
  declare_parameter("topics.status", "slam/status");
  declare_parameter("topics.save_map_service", "slam/save_map");
  declare_parameter("topics.load_map_service", "slam/load_map");
  declare_parameter("topics.odom", std::string("slam/odometry"));
  declare_parameter("odom_pose_cov", odom_pose_cov_);
  declare_parameter("topics.pose", std::string("slam/pose"));

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
  get_parameter("prior.pose_cov", prior_pose_cov_);
  get_parameter("odom_pose_cov", odom_pose_cov_);

  std::string status_topic, save_map_service, load_map_service;
  get_parameter("topics.status", status_topic);
  get_parameter("topics.save_map_service", save_map_service);
  get_parameter("topics.load_map_service", load_map_service);

  std::string odom_output_topic;
  get_parameter("topics.odom", odom_output_topic);

  // Core components
  pose_graph_ = std::make_unique<PoseGraph>();
  map_manager_ = std::make_unique<MapManager>();

  std::string pose_topic;
  get_parameter("topics.pose", pose_topic);

  // Publishers
  status_pub_ = create_publisher<eidos_msgs::msg::SlamStatus>(status_topic, 1);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic, 10);

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
  motion_model_loader_ = std::make_unique<pluginlib::ClassLoader<MotionModelPlugin>>(
      "eidos", "eidos::MotionModelPlugin");
  reloc_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<RelocalizationPlugin>>(
      "eidos", "eidos::RelocalizationPlugin");
  vis_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<VisualizationPlugin>>(
      "eidos", "eidos::VisualizationPlugin");

  loadFactorPlugins();
  loadMotionModel();
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

  // Always enter WARMING_UP
  state_ = SlamState::WARMING_UP;
  RCLCPP_INFO(get_logger(), "\033[33m[WARMING_UP]\033[0m Waiting for factors...");

  // Activate lifecycle publishers
  status_pub_->on_activate();
  pose_pub_->on_activate();
  odom_pub_->on_activate();

  // Activate plugins
  for (auto& plugin : factor_plugins_) {
    plugin->activate();
  }
  if (motion_model_) {
    motion_model_->activate();
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

  status_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  odom_pub_->on_deactivate();

  for (auto& plugin : factor_plugins_) {
    plugin->deactivate();
  }
  if (motion_model_) {
    motion_model_->deactivate();
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
  motion_model_.reset();
  reloc_plugins_.clear();
  vis_plugins_.clear();
  factor_plugin_loader_.reset();
  motion_model_loader_.reset();
  reloc_plugin_loader_.reset();
  vis_plugin_loader_.reset();

  odom_pub_.reset();

  status_pub_.reset();
  pose_pub_.reset();
  save_map_srv_.reset();
  load_map_srv_.reset();

  pose_graph_.reset();
  map_manager_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  // Reset SLAM state
  state_ = SlamState::INITIALIZING;
  current_pose_ = gtsam::Pose3();
  current_keygroup_ = -1;
  std::fill(std::begin(current_transform_), std::end(current_transform_), 0.0f);
  loop_closure_detected_ = false;
  accumulated_graph_ = gtsam::NonlinearFactorGraph();
  accumulated_values_ = gtsam::Values();

  RCLCPP_INFO(get_logger(), "\033[36m[CLEANED_UP]\033[0m Eidos SLAM cleaned up");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "\033[36m[SHUTTING_DOWN]\033[0m ...");

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

  if (run_vis) {
    for (auto& plugin : vis_plugins_) {
      plugin->onOptimizationComplete(vis_values, vis_loop_closure);
    }
  }
}

void SlamCore::handleInitializing() {
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

  // Also check motion model readiness
  bool motion_ready = true;
  if (motion_model_) {
    motion_ready = motion_model_->isReady();
    total++;
    if (motion_ready) ready_count++;
    std::string status = motion_model_->getReadyStatus();
    report += "\n  " + motion_model_->getName() + " (motion): "
           + (motion_ready ? "\033[32mREADY\033[0m" : "\033[33mWAITING\033[0m");
    if (!status.empty()) report += " — " + status;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "\033[33m[WARMING_UP]\033[0m plugins ready: %d/%d%s",
      ready_count, total, report.c_str());

  if (ready_count < total) return;

  RCLCPP_INFO(get_logger(),
      "\033[32m[READY]\033[0m All plugins ready");

  if (!reloc_plugins_.empty() && map_manager_->hasPriorMap()) {
    state_ = SlamState::RELOCALIZING;
    relocalization_start_time_ = timestamp;
    RCLCPP_INFO(get_logger(), "\033[35m[RELOCALIZING]\033[0m Prior map loaded, entering relocalization");
  } else {
    // Use gravity-aligned orientation from motion model if available
    gtsam::Pose3 initial_pose = gtsam::Pose3::Identity();
    if (motion_model_) {
      auto initial_rot = motion_model_->getInitialOrientation();
      if (initial_rot.has_value()) {
        initial_pose = gtsam::Pose3(initial_rot.value(), gtsam::Point3(0, 0, 0));
      }
    }
    beginTracking(initial_pose, timestamp);
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
  current_keygroup_ = 0;

  // Create first keyframe with prior factor
  // State 0 uses index 255 (reserved for init/prior state)
  gtsam::Key key0 = gtsam::Symbol(255, 0);

  gtsam::NonlinearFactorGraph initial_factors;
  gtsam::Values initial_values;
  initial_values.insert(key0, current_pose_);

  auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << prior_pose_cov_[0], prior_pose_cov_[1], prior_pose_cov_[2],
       prior_pose_cov_[3], prior_pose_cov_[4], prior_pose_cov_[5]).finished());
  initial_factors.addPrior(key0, current_pose_, prior_noise);

  // Motion model state 0 initialization (e.g., IMU: V(0), B(0) priors)
  if (motion_model_) {
    motion_model_->onStateZero(key0, timestamp, current_pose_,
                                initial_factors, initial_values);
  }

  // Collect factors from all factor plugins for state 0
  // Same keygroup logic as handleTracking: plugins may contribute states
  struct KgState { gtsam::Key key; double t; };
  std::vector<KgState> keygroup_states;

  for (size_t i = 0; i < factor_plugins_.size(); i++) {
    gtsam::Key plugin_key = gtsam::Symbol(static_cast<unsigned char>(i), 0);
    auto plugin_result = factor_plugins_[i]->getFactors(plugin_key);

    if (plugin_result.timestamp.has_value()) {
      // Plugin wants to create a state — insert initial pose value
      initial_values.insert(plugin_key, current_pose_);
      keygroup_states.push_back({plugin_key, plugin_result.timestamp.value()});
    }

    for (auto& f : plugin_result.factors) initial_factors.add(f);
    initial_values.insert(plugin_result.values);
  }

  // Connect plugin states to key0 via motion model (sorted by timestamp)
  std::sort(keygroup_states.begin(), keygroup_states.end(),
            [](auto& a, auto& b) { return a.t < b.t; });

  // Build chain: key0 → first plugin state → ... → last plugin state
  gtsam::Key chain_prev = key0;
  double chain_prev_t = timestamp;
  for (auto& ks : keygroup_states) {
    if (motion_model_) {
      motion_model_->generateMotionModel(
          chain_prev, chain_prev_t, ks.key, ks.t,
          initial_factors, initial_values);
    }
    chain_prev = ks.key;
    chain_prev_t = ks.t;
  }

  // Track last state for motion model chaining
  last_keygroup_state_ = {chain_prev, chain_prev_t};

  // Store and optimize
  accumulated_graph_.add(initial_factors);
  if (!accumulated_values_.exists(key0)) {
    accumulated_values_.insert(key0, current_pose_);
  }
  for (auto& ks : keygroup_states) {
    if (!accumulated_values_.exists(ks.key)) {
      accumulated_values_.insert(ks.key, current_pose_);
    }
  }
  auto optimized = pose_graph_->update(initial_factors, initial_values);

  // Store keyframes — key0 first, then plugin states
  PoseType pose_6d = gtsamPose3ToPoseType(current_pose_, timestamp);
  pose_6d.intensity = 0.0f;
  map_manager_->addKeyframe(key0, pose_6d);

  for (auto& ks : keygroup_states) {
    gtsam::Pose3 pose = optimized.exists(ks.key)
        ? optimized.at<gtsam::Pose3>(ks.key) : current_pose_;
    PoseType kf_pose = gtsamPose3ToPoseType(pose, ks.t);
    kf_pose.intensity = static_cast<float>(map_manager_->numKeyframes());
    map_manager_->addKeyframe(ks.key, kf_pose);
  }

  // Set state before notifying plugins
  state_ = SlamState::TRACKING;

  // Notify all factors of initial optimization
  for (auto& plugin : factor_plugins_) {
    plugin->onOptimizationComplete(optimized, false);
  }
  if (motion_model_) {
    motion_model_->onOptimizationComplete(optimized, false);
  }

  // Notify visualizers
  for (auto& plugin : vis_plugins_) {
    plugin->onOptimizationComplete(optimized, false);
  }

  publishPose();
  publishOdometry();
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
  // 1. Get pose estimate via motion model delta from last optimized pose
  //    The motion model's absolute pose is only used for EKF odometry downstream.
  //    For graph initial values, we use: last_optimized * delta(mm).
  std::optional<gtsam::Pose3> best_pose;

  if (motion_model_) {
    auto mm_pose = motion_model_->getCurrentPose();
    if (mm_pose.has_value()) {
      if (has_optimization_anchor_) {
        // Relative delta: last_optimized * (mm_at_last_opt⁻¹ * mm_now)
        gtsam::Pose3 delta = mm_pose_at_last_optimization_.between(*mm_pose);
        best_pose = last_optimized_pose_.compose(delta);
      } else {
        best_pose = mm_pose;  // No anchor yet — use absolute (first keygroup only)
      }
    }
  }

  // Fall back to factor plugins
  for (auto& plugin : factor_plugins_) {
    auto pose = plugin->processFrame(timestamp);
    if (pose.has_value() && !best_pose.has_value()) {
      best_pose = pose;
    }
  }

  if (!best_pose.has_value()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "\033[32m[TRACKING]\033[0m Waiting for first pose...");
    return;
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
  if (poses_6d->empty()) {
    return;
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

  // 3. If new state, form a keygroup
  if (new_state) {
    current_keygroup_++;

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;

    // Phase A: Collect factors from each plugin
    struct KgState { gtsam::Key key; double t; };
    std::vector<KgState> keygroup_states;

    loop_closure_detected_ = false;
    std::string factor_summary;

    for (size_t i = 0; i < factor_plugins_.size(); i++) {
      gtsam::Key key = gtsam::Symbol(static_cast<unsigned char>(i), current_keygroup_);
      auto result = factor_plugins_[i]->getFactors(key);

      if (!factor_summary.empty()) factor_summary += ", ";
      factor_summary += factor_plugins_[i]->getName() + ":" + std::to_string(result.factors.size());

      if (result.timestamp.has_value()) {
        // This plugin contributes a state at its sensor timestamp.
        // Use the plugin's own initial value if provided (e.g. GICP-matched pose),
        // otherwise fall back to the motion model pose.
        if (result.values.exists(key)) {
          new_values.insert(key, result.values.at<gtsam::Pose3>(key));
        } else {
          new_values.insert(key, current_pose_);
        }
        keygroup_states.push_back({key, result.timestamp.value()});
      }

      for (auto& f : result.factors) {
        new_factors.add(f);
        // Detect loop closure: BetweenFactor connecting keys from distant keygroups
        auto between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f);
        if (between) {
          gtsam::Symbol s1(between->key1()), s2(between->key2());
          int gap = std::abs(static_cast<int>(s1.index()) -
                             static_cast<int>(s2.index()));
          if (gap > 1) {
            loop_closure_detected_ = true;
          }
        }
      }
      // Insert any additional values from the plugin (e.g. bias keys),
      // skipping the pose key already handled above.
      for (const auto& kv : result.values) {
        if (!new_values.exists(kv.key)) {
          new_values.insert(kv.key, kv.value);
        }
      }
    }

    // Phase B: Motion model — connect consecutive states by timestamp
    std::sort(keygroup_states.begin(), keygroup_states.end(),
              [](auto& a, auto& b) { return a.t < b.t; });

    // Motion model only connects states WITHIN this keygroup (intra-keygroup).
    // Inter-keygroup connectivity comes from factor plugins (LISO BetweenFactors,
    // GPS absolute constraints). No motion model chaining between keygroups.
    if (!keygroup_states.empty()) {
      for (size_t i = 0; i + 1 < keygroup_states.size(); i++) {
        motion_model_->generateMotionModel(
            keygroup_states[i].key, keygroup_states[i].t,
            keygroup_states[i+1].key, keygroup_states[i+1].t,
            new_factors, new_values);
      }
    }

    // Track accumulated graph for serialization
    accumulated_graph_.add(new_factors);
    for (const auto& ks : keygroup_states) {
      if (!accumulated_values_.exists(ks.key)) {
        accumulated_values_.insert(ks.key, current_pose_);
      }
    }

    // Optimize
    auto optimized = pose_graph_->update(new_factors, new_values);

    // Handle loop closure
    if (loop_closure_detected_) {
      pose_graph_->updateExtra(5);
      optimized = pose_graph_->getOptimizedValues();
      map_manager_->updatePoses(optimized);
    }

    // Store keyframes for each state in this keygroup
    for (auto& ks : keygroup_states) {
      gtsam::Pose3 pose;
      if (optimized.exists(ks.key)) {
        pose = optimized.at<gtsam::Pose3>(ks.key);
      } else {
        pose = current_pose_;
      }
      PoseType pose_6d = gtsamPose3ToPoseType(pose, ks.t);
      pose_6d.intensity = static_cast<float>(map_manager_->numKeyframes());
      map_manager_->addKeyframe(ks.key, pose_6d);
    }

    // Get latest optimized pose (from last state in keygroup)
    // and anchor the motion model delta tracking
    if (!keygroup_states.empty()) {
      auto last_key = keygroup_states.back().key;
      if (optimized.exists(last_key)) {
        auto latest_pose = optimized.at<gtsam::Pose3>(last_key);
        current_pose_ = latest_pose;
        current_transform_[0] = static_cast<float>(latest_pose.rotation().roll());
        current_transform_[1] = static_cast<float>(latest_pose.rotation().pitch());
        current_transform_[2] = static_cast<float>(latest_pose.rotation().yaw());
        current_transform_[3] = static_cast<float>(latest_pose.translation().x());
        current_transform_[4] = static_cast<float>(latest_pose.translation().y());
        current_transform_[5] = static_cast<float>(latest_pose.translation().z());

        // Anchor: next cycle computes current_pose = latest_pose * delta(mm)
        last_optimized_pose_ = latest_pose;
        if (motion_model_) {
          auto mm_now = motion_model_->getCurrentPose();
          if (mm_now.has_value()) {
            mm_pose_at_last_optimization_ = *mm_now;
          }
        }
        has_optimization_anchor_ = true;
      }
    }

    RCLCPP_INFO(get_logger(),
        "\033[32m[TRACKING]\033[0m keygroup=%d states=%zu optimized: (%.3f, %.3f, %.3f) factors: %zu [%s]",
        current_keygroup_, keygroup_states.size(),
        current_pose_.translation().x(), current_pose_.translation().y(),
        current_pose_.translation().z(), new_factors.size(), factor_summary.c_str());

    // Notify plugins of optimization result
    for (auto& plugin : factor_plugins_) {
      plugin->onOptimizationComplete(optimized, loop_closure_detected_);
    }
    if (motion_model_) {
      motion_model_->onOptimizationComplete(optimized, loop_closure_detected_);
    }

    // Signal visualization
    run_vis = true;
    vis_values = optimized;
    vis_loop_closure = loop_closure_detected_;

    publishPose();

    // Only publish graph-optimized odometry when the graph actually grew
    if (!keygroup_states.empty()) {
      publishOdometry();
    }
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
      RCLCPP_INFO(get_logger(), "Loaded factor plugin [%zu]: %s (%s)",
                  factor_plugins_.size() - 1, name.c_str(), plugin_type.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load factor plugin '%s' (%s): %s",
                   name.c_str(), plugin_type.c_str(), ex.what());
    }
  }
}

void SlamCore::loadMotionModel() {
  std::string motion_model_type;
  get_parameter("motion_model.plugin", motion_model_type);

  if (motion_model_type.empty()) {
    RCLCPP_WARN(get_logger(), "No motion model configured. Using HolonomicMotionModel.");
    motion_model_type = "eidos::HolonomicMotionModel";
  }

  try {
    motion_model_ = motion_model_loader_->createSharedInstance(motion_model_type);
    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    motion_model_->initialize(this, "motion_model", shared_from_this(),
                               tf_buffer_.get(), cb_group);
    RCLCPP_INFO(get_logger(), "Loaded motion model: %s", motion_model_type.c_str());
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to load motion model '%s': %s",
                 motion_model_type.c_str(), ex.what());
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
  msg.current_state_index = current_keygroup_;
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

void SlamCore::publishPose() {
  if (pose_pub_->get_subscription_count() == 0) return;

  auto q = current_pose_.rotation().toQuaternion();

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.pose.position.x = current_pose_.translation().x();
  msg.pose.position.y = current_pose_.translation().y();
  msg.pose.position.z = current_pose_.translation().z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  pose_pub_->publish(msg);
}

void SlamCore::publishOdometry() {
  if (!odom_pub_->is_activated()) return;

  auto q = current_pose_.rotation().toQuaternion();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.child_frame_id = base_link_frame_;
  msg.pose.pose.position.x = current_pose_.translation().x();
  msg.pose.pose.position.y = current_pose_.translation().y();
  msg.pose.pose.position.z = current_pose_.translation().z();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Pose covariance diagonal (6x6 row-major: x, y, z, roll, pitch, yaw)
  msg.pose.covariance[0]  = odom_pose_cov_[0];  // x
  msg.pose.covariance[7]  = odom_pose_cov_[1];  // y
  msg.pose.covariance[14] = odom_pose_cov_[2];  // z
  msg.pose.covariance[21] = odom_pose_cov_[3];  // roll
  msg.pose.covariance[28] = odom_pose_cov_[4];  // pitch
  msg.pose.covariance[35] = odom_pose_cov_[5];  // yaw

  odom_pub_->publish(msg);
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

int SlamCore::getCurrentKeygroup() const {
  return current_keygroup_;
}

const gtsam::NonlinearFactorGraph& SlamCore::getAccumulatedGraph() const {
  return accumulated_graph_;
}

std::optional<gtsam::Pose3> SlamCore::getMotionModelPose() const {
  return motion_model_ ? motion_model_->getCurrentPose() : std::nullopt;
}

}  // namespace eidos
