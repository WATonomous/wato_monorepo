#include "eidos/slam_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  declare_parameter("relocalization_timeout", relocalization_timeout_);
  declare_parameter("frames.map", map_frame_);
  declare_parameter("frames.odometry", odom_frame_);
  declare_parameter("frames.base_link", base_link_frame_);
  declare_parameter("transforms.map_to_odom.publish", publish_map_to_odom_);
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
  declare_parameter("isam2.update_iterations", isam2_update_iterations_);
  declare_parameter("isam2.correction_iterations", isam2_correction_iterations_);
  declare_parameter("isam2.relinearize_threshold", isam2_relinearize_threshold_);
  declare_parameter("isam2.relinearize_skip", isam2_relinearize_skip_);

  get_parameter("slam_rate", slam_rate_);
  get_parameter("relocalization_timeout", relocalization_timeout_);
  get_parameter("frames.map", map_frame_);
  get_parameter("frames.odometry", odom_frame_);
  get_parameter("frames.base_link", base_link_frame_);
  get_parameter("transforms.map_to_odom.publish", publish_map_to_odom_);
  get_parameter("map.load_directory", map_load_directory_);
  get_parameter("map.save_directory", map_save_directory_);
  get_parameter("prior.pose_cov", prior_pose_cov_);
  get_parameter("odom_pose_cov", odom_pose_cov_);
  get_parameter("isam2.update_iterations", isam2_update_iterations_);
  get_parameter("isam2.correction_iterations", isam2_correction_iterations_);
  get_parameter("isam2.relinearize_threshold", isam2_relinearize_threshold_);
  get_parameter("isam2.relinearize_skip", isam2_relinearize_skip_);

  std::string status_topic, save_map_service, load_map_service;
  get_parameter("topics.status", status_topic);
  get_parameter("topics.save_map_service", save_map_service);
  get_parameter("topics.load_map_service", load_map_service);

  std::string odom_output_topic;
  get_parameter("topics.odom", odom_output_topic);

  // TF broadcaster for map -> odom
  if (publish_map_to_odom_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // Core components
  pose_graph_ = std::make_unique<PoseGraph>();
  pose_graph_->reset(isam2_relinearize_threshold_, isam2_relinearize_skip_);
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

  // Check for colliding TF publishers — no two plugins should broadcast the same frame pair
  {
    // Collect (frame_id, child_frame_id) -> plugin name
    std::map<std::pair<std::string, std::string>, std::vector<std::string>> tf_map;
    for (const auto& plugin : factor_plugins_) {
      if (plugin->publishesTf()) {
        auto frames = plugin->getTfFrames();
        if (!frames.first.empty() && !frames.second.empty()) {
          tf_map[frames].push_back(plugin->getName());
        }
      }
    }
    if (motion_model_ && motion_model_->publishesTf()) {
      auto frames = motion_model_->getTfFrames();
      if (!frames.first.empty() && !frames.second.empty()) {
        tf_map[frames].push_back(motion_model_->getName());
      }
    }

    for (const auto& [frames, publishers] : tf_map) {
      if (publishers.size() > 1) {
        RCLCPP_ERROR(get_logger(),
            "\033[31m[CONFIG ERROR]\033[0m Multiple plugins broadcast TF %s -> %s:",
            frames.first.c_str(), frames.second.c_str());
        for (const auto& name : publishers) {
          RCLCPP_ERROR(get_logger(), "  - %s", name.c_str());
        }
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(get_logger(), "\033[36m[TF]\033[0m %s -> %s broadcast by: %s",
                  frames.first.c_str(), frames.second.c_str(),
                  publishers[0].c_str());
    }
  }

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

  // Start visualization timer — separate callback group so it never blocks SLAM
  vis_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  vis_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlamCore::visLoop, this),
      vis_callback_group_);

  RCLCPP_INFO(get_logger(), "\033[36m[ACTIVATED]\033[0m Eidos SLAM active");
  return CallbackReturn::SUCCESS;
}

SlamCore::CallbackReturn SlamCore::on_deactivate(
    const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "\033[36m[DEACTIVATING]\033[0m ...");

  if (vis_timer_) {
    vis_timer_->cancel();
    vis_timer_.reset();
  }
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

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  // Reset SLAM state
  state_ = SlamState::INITIALIZING;
  current_pose_ = gtsam::Pose3();
  next_state_index_ = 0;
  last_state_key_ = 0;
  last_state_timestamp_ = 0.0;
  last_state_owner_.clear();
  has_last_state_ = false;
  has_optimization_anchor_ = false;
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

// ---- TF broadcasting ----

void SlamCore::broadcastMapToOdom() {
  if (!publish_map_to_odom_ || !tf_broadcaster_) return;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = odom_frame_;
  auto q = cached_map_to_odom_.rotation().toQuaternion();
  tf.transform.translation.x = cached_map_to_odom_.translation().x();
  tf.transform.translation.y = cached_map_to_odom_.translation().y();
  tf.transform.translation.z = cached_map_to_odom_.translation().z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);
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
    if (state_ == SlamState::TRACKING) {
      broadcastMapToOdom();
    }
  }

  // Hand off to vis timer — never block the SLAM loop
  if (run_vis) {
    std::lock_guard<std::mutex> lock(vis_mtx_);
    vis_values_ = std::move(vis_values);
    vis_loop_closure_ = vis_loop_closure;
    vis_pending_ = true;
  }
}

void SlamCore::visLoop() {
  gtsam::Values values;
  bool loop_closure = false;

  {
    std::lock_guard<std::mutex> lock(vis_mtx_);
    if (!vis_pending_) return;
    values = std::move(vis_values_);
    loop_closure = vis_loop_closure_;
    vis_pending_ = false;
  }

  for (auto& plugin : vis_plugins_) {
    plugin->onOptimizationComplete(values, loop_closure);
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

void SlamCore::beginTracking(const gtsam::Pose3& initial_pose, double /*timestamp*/) {
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

  // No state created here.  The first plugin to produce data via
  // hasData()/getFactors() creates state 0 through the normal
  // handleTracking() path.  handleTracking() detects the very first
  // state (!has_last_state_) and adds the PriorFactor + motion model init.
  next_state_index_ = 0;
  has_last_state_ = false;
  last_state_owner_.clear();

  state_ = SlamState::TRACKING;

  publishPose();
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
  auto t0 = std::chrono::steady_clock::now();
  auto elapsed_ms = [&t0]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  };

  // 1. Get pose estimate via motion model delta from last optimized pose
  std::optional<gtsam::Pose3> best_pose;

  if (motion_model_) {
    auto mm_pose = motion_model_->getCurrentPose();
    if (mm_pose.has_value()) {
      if (has_optimization_anchor_) {
        gtsam::Pose3 delta = mm_pose_at_last_optimization_.between(*mm_pose);
        best_pose = last_optimized_pose_.compose(delta);
      } else {
        best_pose = mm_pose;
      }
    }
  }

  double t_pose_est = elapsed_ms();

  // Fall back to factor plugins for pose estimate
  for (auto& plugin : factor_plugins_) {
    auto pose = plugin->processFrame(timestamp);
    if (pose.has_value() && !best_pose.has_value()) {
      best_pose = pose;
    }
  }

  double t_process_frame = elapsed_ms();

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

  // 2. Collect new states from all plugins that have data
  struct NewState {
    gtsam::Key key;
    double timestamp;
    StampedFactorResult result;
    std::string owner;
  };
  std::vector<NewState> new_states;

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  loop_closure_detected_ = false;

  for (size_t i = 0; i < factor_plugins_.size(); i++) {
    if (!factor_plugins_[i]->hasData()) continue;

    gtsam::Key key = gtsam::Symbol('x', next_state_index_);
    auto result = factor_plugins_[i]->getFactors(key);

    if (result.timestamp.has_value()) {
      // Plugin creates a state at its sensor timestamp
      next_state_index_++;
      new_states.push_back({key, result.timestamp.value(),
                            std::move(result), factor_plugins_[i]->getName()});
    } else {
      // No new state — add factors directly (loop closure, bias priors, etc.)
      for (auto& f : result.factors) {
        new_factors.add(f);
        // BetweenFactor<Pose3> without a new state = loop closure
        auto between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f);
        if (between) {
          loop_closure_detected_ = true;
        }
      }
      for (const auto& kv : result.values) {
        if (!new_values.exists(kv.key)) {
          new_values.insert(kv.key, kv.value);
        }
      }
    }
  }

  double t_collect = elapsed_ms();

  // 3. Nothing to optimize this tick
  if (new_states.empty() && new_factors.empty()) {
    publishPose();
    publishOdometry();
    return;
  }

  // 4. Sort new states by timestamp
  std::sort(new_states.begin(), new_states.end(),
            [](auto& a, auto& b) { return a.timestamp < b.timestamp; });

  // 5. Chain with motion model and collect factors/values
  std::string factor_summary;
  for (auto& ns : new_states) {
    // Insert initial value for the pose state
    if (ns.result.values.exists(ns.key)) {
      new_values.insert(ns.key, ns.result.values.at<gtsam::Pose3>(ns.key));
    } else {
      new_values.insert(ns.key, current_pose_);
    }

    // Add factors from this plugin
    for (auto& f : ns.result.factors) {
      new_factors.add(f);
    }

    // Add any additional values (bias keys, etc.)
    for (const auto& kv : ns.result.values) {
      if (!new_values.exists(kv.key)) {
        new_values.insert(kv.key, kv.value);
      }
    }

    // Let latching plugins attach factors to this state
    for (auto& plugin : factor_plugins_) {
      auto latch = plugin->latchFactors(ns.key, ns.timestamp);
      for (auto& f : latch.factors) {
        new_factors.add(f);
      }
      for (const auto& kv : latch.values) {
        if (!new_values.exists(kv.key)) {
          new_values.insert(kv.key, kv.value);
        }
      }
      if (!latch.factors.empty()) {
        if (!factor_summary.empty()) factor_summary += ", ";
        factor_summary += plugin->getName() + "(latch):" + std::to_string(latch.factors.size());
        // GPS (or any latched factor) triggers extra iterations like a loop closure
        loop_closure_detected_ = true;
      }
    }

    // First state ever: anchor with PriorFactor + motion model init
    if (!has_last_state_) {
      gtsam::Pose3 anchor_pose = ns.result.values.exists(ns.key)
          ? ns.result.values.at<gtsam::Pose3>(ns.key) : current_pose_;
      // Config is [x, y, z, roll, pitch, yaw], GTSAM expects [rot, rot, rot, trans, trans, trans]
      auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << prior_pose_cov_[3], prior_pose_cov_[4], prior_pose_cov_[5],
           prior_pose_cov_[0], prior_pose_cov_[1], prior_pose_cov_[2]).finished());
      new_factors.addPrior(ns.key, anchor_pose, prior_noise);

      if (motion_model_) {
        motion_model_->onStateZero(ns.key, ns.timestamp, anchor_pose,
                                    new_factors, new_values);
      }
    }

    // Motion model: only connect cross-sensor state transitions.
    // Same-plugin consecutive states are already constrained by the plugin's
    // own factors (e.g. LISO BetweenFactors). The motion model bridges
    // sensor A → sensor B transitions.
    if (has_last_state_ && motion_model_ && last_state_owner_ != ns.owner) {
      motion_model_->generateMotionModel(
          last_state_key_, last_state_timestamp_,
          ns.key, ns.timestamp,
          new_factors, new_values);
    }

    // Update last state tracking
    last_state_key_ = ns.key;
    last_state_timestamp_ = ns.timestamp;
    last_state_owner_ = ns.owner;
    has_last_state_ = true;

    if (!factor_summary.empty()) factor_summary += ", ";
    factor_summary += ns.owner + ":" + std::to_string(ns.result.factors.size());
  }

  double t_chain = elapsed_ms();

  // 6. Track accumulated graph for serialization
  accumulated_graph_.add(new_factors);
  map_manager_->addEdges(new_factors);
  for (const auto& ns : new_states) {
    if (!accumulated_values_.exists(ns.key)) {
      accumulated_values_.insert(ns.key, current_pose_);
    }
  }

  // 7. Optimize
  auto optimized = pose_graph_->update(new_factors, new_values, isam2_update_iterations_);
  double t_optimize = elapsed_ms();

  // 8. Handle loop closure / GPS correction (extra iterations for convergence)
  if (loop_closure_detected_) {
    pose_graph_->updateExtra(isam2_correction_iterations_);
    optimized = pose_graph_->getOptimizedValues();
  }

  // Update all keyframe poses with latest optimized values
  // (GPS corrections, loop closures, etc. shift existing states)
  map_manager_->updatePoses(optimized);

  // 9. Store keyframes for each new state
  for (auto& ns : new_states) {
    gtsam::Pose3 pose = optimized.exists(ns.key)
        ? optimized.at<gtsam::Pose3>(ns.key) : current_pose_;
    PoseType pose_6d = gtsamPose3ToPoseType(pose, ns.timestamp);
    pose_6d.intensity = static_cast<float>(map_manager_->numKeyframes());
    map_manager_->addKeyframe(ns.key, pose_6d, ns.owner);
  }

  double t_keyframes = elapsed_ms();

  // 10. Update current pose from latest optimized state
  if (!new_states.empty()) {
    auto last_key = new_states.back().key;
    if (optimized.exists(last_key)) {
      auto latest_pose = optimized.at<gtsam::Pose3>(last_key);
      current_pose_ = latest_pose;
      current_transform_[0] = static_cast<float>(latest_pose.rotation().roll());
      current_transform_[1] = static_cast<float>(latest_pose.rotation().pitch());
      current_transform_[2] = static_cast<float>(latest_pose.rotation().yaw());
      current_transform_[3] = static_cast<float>(latest_pose.translation().x());
      current_transform_[4] = static_cast<float>(latest_pose.translation().y());
      current_transform_[5] = static_cast<float>(latest_pose.translation().z());

      last_optimized_pose_ = latest_pose;
      if (motion_model_) {
        auto mm_now = motion_model_->getCurrentPose();
        if (mm_now.has_value()) {
          mm_pose_at_last_optimization_ = *mm_now;
        }
      }
      has_optimization_anchor_ = true;

      // Update cached map->odom correction from the latest optimization
      if (publish_map_to_odom_) {
        try {
          auto odom_base_tf = tf_buffer_->lookupTransform(
              odom_frame_, base_link_frame_, tf2::TimePointZero);
          const auto& tb = odom_base_tf.transform.translation;
          const auto& rb = odom_base_tf.transform.rotation;
          gtsam::Pose3 T_odom_base(
              gtsam::Rot3(Eigen::Quaterniond(rb.w, rb.x, rb.y, rb.z).toRotationMatrix()),
              gtsam::Point3(tb.x, tb.y, tb.z));
          // T_map_odom = T_map_base * inv(T_odom_base)
          cached_map_to_odom_ = latest_pose.compose(T_odom_base.inverse());
          broadcastMapToOdom();
        } catch (const tf2::TransformException&) {
          // odom->base_link not yet available — keep previous cached value (identity until first update)
        }
      }
    }
  }

  RCLCPP_INFO(get_logger(),
      "\033[32m[TRACKING]\033[0m state_idx=%lu states=%zu pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) factors=%zu [%s]",
      next_state_index_, new_states.size(),
      current_pose_.translation().x(), current_pose_.translation().y(),
      current_pose_.translation().z(),
      current_pose_.rotation().roll(), current_pose_.rotation().pitch(),
      current_pose_.rotation().yaw(),
      new_factors.size(), factor_summary.c_str());

  // 11. Notify factor plugins of optimization result
  for (size_t i = 0; i < factor_plugins_.size(); i++) {
    double t_before = elapsed_ms();
    factor_plugins_[i]->onOptimizationComplete(optimized, loop_closure_detected_);
    double t_after = elapsed_ms();
    RCLCPP_INFO(get_logger(),
        "\033[33m[TIMING]\033[0m plugin_notify[%s]=%.1fms",
        factor_plugins_[i]->getName().c_str(), t_after - t_before);
  }

  double t_total = elapsed_ms();

  RCLCPP_INFO(get_logger(),
      "\033[33m[TIMING]\033[0m total=%.1fms | pose_est=%.1f | processFrame=%.1f | "
      "collect=%.1f | chain=%.1f | optimize=%.1f | keyframes=%.1f | notify=%.1f",
      t_total, t_pose_est, t_process_frame - t_pose_est,
      t_collect - t_process_frame, t_chain - t_collect,
      t_optimize - t_chain, t_keyframes - t_optimize,
      t_total - t_keyframes);

  // Signal visualization
  run_vis = true;
  vis_values = optimized;
  vis_loop_closure = loop_closure_detected_;

  publishPose();
  publishOdometry();
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
  msg.current_state_index = static_cast<int>(next_state_index_);
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

uint64_t SlamCore::getCurrentStateIndex() const {
  return next_state_index_;
}

const gtsam::NonlinearFactorGraph& SlamCore::getAccumulatedGraph() const {
  return accumulated_graph_;
}

std::optional<gtsam::Pose3> SlamCore::getMotionModelPose() const {
  return motion_model_ ? motion_model_->getCurrentPose() : std::nullopt;
}

std::optional<Eigen::MatrixXd> SlamCore::getLatestPoseCovariance() const {
  if (!has_last_state_) return std::nullopt;
  try {
    return pose_graph_->getMarginalCovariance(last_state_key_);
  } catch (...) {
    return std::nullopt;
  }
}

}  // namespace eidos
