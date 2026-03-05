#include "eidos/plugins/factors/liso_factor.hpp"

#include <queue>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/slam_core.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void LisoFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // Declare parameters
  node_->declare_parameter(prefix + ".lidar_topic", "/lidar/points");
  node_->declare_parameter(prefix + ".odom_topic", "liso/odometry");
  node_->declare_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->declare_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".submap_radius", submap_radius_);
  node_->declare_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".min_inliers", min_inliers_);
  node_->declare_parameter(prefix + ".min_noise", min_noise_);
  node_->declare_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->declare_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->declare_parameter(prefix + ".lidar_frame", lidar_frame_);

  // Read parameters
  std::string lidar_topic, odom_topic;
  node_->get_parameter(prefix + ".lidar_topic", lidar_topic);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->get_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".submap_radius", submap_radius_);
  node_->get_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_iterations", max_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".min_inliers", min_inliers_);
  node_->get_parameter(prefix + ".min_noise", min_noise_);
  node_->get_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->get_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->get_parameter(prefix + ".lidar_frame", lidar_frame_);

  node_->get_parameter("frames.map", map_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  // Create subscription
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&LisoFactor::lidarCallback, this, std::placeholders::_1),
      sub_opts);

  // Create odometry publisher
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (GICP scan-to-submap)", name_.c_str());
}

void LisoFactor::activate() {
  active_ = true;
  odom_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void LisoFactor::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void LisoFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  first_scan_ = true;
  scan_received_ = false;
  has_tf_ = false;
  has_last_factor_ = false;
  has_prev_liso_ = false;
  has_last_match_ = false;

  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_.reset();
    cached_submap_tree_.reset();
  }
  {
    std::lock_guard lock(result_mtx_);
    has_cached_result_ = false;
  }
}

bool LisoFactor::isReady() const {
  return scan_received_;
}

std::string LisoFactor::getReadyStatus() const {
  return scan_received_ ? "LiDAR data received" : "waiting for LiDAR data";
}

// ---------------------------------------------------------------------------
// LiDAR callback — scan matching + odometry publishing (~27 Hz)
// ---------------------------------------------------------------------------
void LisoFactor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!active_) return;

  if (!scan_received_) {
    scan_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] first LiDAR scan received (%zu points)",
                name_.c_str(), static_cast<size_t>(msg->width * msg->height));
  }

  // Don't process scans until TRACKING — IMU needs to calibrate gravity first
  if (core_->getState() != SlamState::TRACKING) return;

  // Look up static TF: base_link ← lidar (once)
  if (!has_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, lidar_frame_,
                                          tf2::TimePointZero);
      const auto& t = tf_msg.transform.translation;
      const auto& r = tf_msg.transform.rotation;
      T_base_lidar_ = Eigen::Isometry3d::Identity();
      T_base_lidar_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      T_base_lidar_.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_tf_ = true;
      RCLCPP_INFO(node_->get_logger(), "[%s] got TF %s <- %s",
                  name_.c_str(), base_link_frame_.c_str(), lidar_frame_.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "[%s] waiting for TF %s <- %s: %s",
                            name_.c_str(), base_link_frame_.c_str(),
                            lidar_frame_.c_str(), ex.what());
      return;
    }
  }

  // Convert ROS msg to small_gicp cloud
  auto raw_cloud = fromRosMsg(*msg);
  if (raw_cloud->empty()) return;

  // Transform points from lidar frame to body frame BEFORE preprocessing,
  // so normals and covariances are computed in body frame (consistent with
  // the frame used for scan matching). Without this, GICP uses lidar-frame
  // normals against world-frame submap normals, causing systematic drift.
  auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  pcl_cloud->reserve(raw_cloud->size());
  for (size_t i = 0; i < raw_cloud->size(); i++) {
    Eigen::Vector4d& pt = raw_cloud->point(i);
    Eigen::Vector3d p = T_base_lidar_ * pt.head<3>();
    pt.head<3>() = p;

    PointType pcl_pt;
    pcl_pt.x = static_cast<float>(p.x());
    pcl_pt.y = static_cast<float>(p.y());
    pcl_pt.z = static_cast<float>(p.z());
    pcl_pt.intensity = 0.0f;
    pcl_cloud->push_back(pcl_pt);
  }

  // Preprocess: downsample + normals + covariances (now in body frame)
  auto [scan, scan_tree] = small_gicp::preprocess_points(
      *raw_cloud, scan_ds_resolution_, num_neighbors_, num_threads_);

  if (scan->empty()) return;

  double scan_time = stamp2Sec(msg->header.stamp);

  // First scan: no submap to match against — just cache
  if (first_scan_) {
    std::lock_guard lock(result_mtx_);
    cached_cloud_ = scan;
    cached_pcl_cloud_ = pcl_cloud;
    cached_timestamp_ = scan_time;

    // Use motion model pose or identity as initial pose
    auto mm_pose = core_->getMotionModelPose();
    cached_pose_ = mm_pose.value_or(gtsam::Pose3::Identity());
    cached_result_ = small_gicp::RegistrationResult();
    cached_result_.converged = true;
    cached_result_.num_inliers = scan->size();
    // Set H to a reasonable default for the first scan
    cached_result_.H = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
    has_cached_result_ = true;
    first_scan_ = false;

    // Seed last match tracking for initial guess computation
    last_matched_pose_ = cached_pose_;
    if (mm_pose.has_value()) {
      last_matched_mm_pose_ = *mm_pose;
    }
    has_last_match_ = true;

    RCLCPP_INFO(node_->get_logger(), "[%s] first scan cached (%zu points), awaiting submap",
                name_.c_str(), scan->size());
    return;
  }

  // Initial guess: last GICP pose + motion model delta since last match
  auto mm_pose = core_->getMotionModelPose();
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (has_last_match_ && mm_pose.has_value()) {
    // delta = last_mm⁻¹ * current_mm (motion model increment since last match)
    gtsam::Pose3 delta = last_matched_mm_pose_.between(*mm_pose);
    gtsam::Pose3 predicted = last_matched_pose_.compose(delta);
    init_guess = Eigen::Isometry3d(predicted.matrix());
  } else if (mm_pose.has_value()) {
    init_guess = Eigen::Isometry3d(mm_pose->matrix());
  }

  // Scan-to-submap matching
  small_gicp::RegistrationResult result;
  {
    std::shared_lock lock(submap_mtx_);
    if (!cached_submap_ || !cached_submap_tree_ || cached_submap_->empty()) {
      return;  // Submap not yet built
    }

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = max_iterations_;
    setting.num_threads = num_threads_;

    result = small_gicp::align(
        *cached_submap_, *scan, *cached_submap_tree_, init_guess, setting);
  }

  // Fitness check
  if (!result.converged || static_cast<int>(result.num_inliers) < min_inliers_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "[%s] scan rejected: converged=%d inliers=%zu",
                          name_.c_str(), result.converged, result.num_inliers);
    return;
  }

  // Extract matched pose
  gtsam::Pose3 matched_pose(gtsam::Rot3(result.T_target_source.rotation()),
                             gtsam::Point3(result.T_target_source.translation()));

  // Cache result atomically for getFactors
  {
    std::lock_guard lock(result_mtx_);
    cached_result_ = result;
    cached_pose_ = matched_pose;
    cached_cloud_ = scan;
    cached_pcl_cloud_ = pcl_cloud;
    cached_timestamp_ = scan_time;
    has_cached_result_ = true;
  }

  // Track last successful match for initial guess computation
  last_matched_pose_ = matched_pose;
  if (mm_pose.has_value()) {
    last_matched_mm_pose_ = *mm_pose;
  }
  has_last_match_ = true;

  // Publish odometry at LiDAR rate
  if (odom_pub_->is_activated()) {
    auto q = matched_pose.rotation().toQuaternion();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = matched_pose.translation().x();
    odom_msg.pose.pose.position.y = matched_pose.translation().y();
    odom_msg.pose.pose.position.z = matched_pose.translation().z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Diagonal covariance [x, y, z, roll, pitch, yaw]
    odom_msg.pose.covariance[0]  = odom_pose_cov_[0];
    odom_msg.pose.covariance[7]  = odom_pose_cov_[1];
    odom_msg.pose.covariance[14] = odom_pose_cov_[2];
    odom_msg.pose.covariance[21] = odom_pose_cov_[3];
    odom_msg.pose.covariance[28] = odom_pose_cov_[4];
    odom_msg.pose.covariance[35] = odom_pose_cov_[5];

    odom_pub_->publish(odom_msg);
  }
}

// ---------------------------------------------------------------------------
// processFrame — return cached GICP pose
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> LisoFactor::processFrame(double /*timestamp*/) {
  std::lock_guard lock(result_mtx_);
  if (has_cached_result_) {
    return cached_pose_;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// getFactors — return BetweenFactor from cached GICP result
// ---------------------------------------------------------------------------
StampedFactorResult LisoFactor::getFactors(gtsam::Key key) {
  StampedFactorResult result;
  if (!active_) return result;

  std::lock_guard lock(result_mtx_);
  if (!has_cached_result_) return result;

  // Distance gate: skip if we haven't moved enough since last injected factor
  gtsam::Point3 current_pos = cached_pose_.translation();
  if (has_last_factor_) {
    double dist = (current_pos - last_factor_position_).norm();
    if (dist < min_scan_distance_) {
      return result;  // no timestamp = no new state, no factors
    }
  }

  // Set timestamp so SlamCore creates a new state
  result.timestamp = cached_timestamp_;

  // Store body-frame clouds in MapManager
  // small_gicp cloud for submap assembly, PCL cloud for visualizers/loop closure
  core_->getMapManager().addKeyframeData(key, "liso_factor/gicp_cloud", cached_cloud_);
  core_->getMapManager().addKeyframeData(key, "liso_factor/cloud", cached_pcl_cloud_);

  // BetweenFactor only: relative LiDAR odometry between consecutive LISO states.
  // No PriorFactor — GICP cannot observe absolute Z on flat ground, so pinning
  // the absolute pose causes a Z-drift feedback loop. GPS and IMU gravity
  // handle absolute positioning; LISO only contributes relative constraints.
  bool added_between = false;
  if (has_prev_liso_) {
    // Build covariance from GICP information matrix H
    Eigen::Matrix<double, 6, 6> cov;
    double det = cached_result_.H.determinant();
    if (std::abs(det) > 1e-10) {
      cov = cached_result_.H.inverse();
    } else {
      cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
    }

    // Apply noise floor to prevent overconfident factors
    for (int i = 0; i < 6; i++) {
      cov(i, i) = std::max(cov(i, i), min_noise_);
    }

    auto noise = gtsam::noiseModel::Gaussian::Covariance(cov);
    gtsam::Pose3 relative_pose = prev_liso_pose_.between(cached_pose_);
    auto between_factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        prev_liso_key_, key, relative_pose, noise);
    result.factors.push_back(between_factor);
    added_between = true;
  }
  // First LISO state: no factors emitted — ImuFactor anchors this state.

  // Update distance gate + relative tracking
  last_factor_position_ = current_pos;
  has_last_factor_ = true;
  prev_liso_key_ = key;
  prev_liso_pose_ = cached_pose_;
  has_prev_liso_ = true;

  gtsam::Symbol sym(key);
  RCLCPP_INFO(node_->get_logger(),
              "\033[36m[%s] %s at (%c,%lu) t=%.3f inliers=%zu\033[0m",
              name_.c_str(),
              added_between ? "BetweenFactor" : "state only (no factors)",
              sym.chr(), sym.index(), cached_timestamp_,
              cached_result_.num_inliers);

  return result;
}

// ---------------------------------------------------------------------------
// onOptimizationComplete — rebuild submap with corrected poses
// ---------------------------------------------------------------------------
void LisoFactor::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool /*loop_closure_detected*/) {
  rebuildSubmap();
}

// ---------------------------------------------------------------------------
// rebuildSubmap — graph BFS from latest state, assemble world-frame submap
// ---------------------------------------------------------------------------
void LisoFactor::rebuildSubmap() {
  if (core_->getState() != SlamState::TRACKING) return;

  const auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();
  if (key_list.empty()) return;

  // Start BFS from the most recent key
  gtsam::Key start_key = key_list.back();

  auto collected_keys = bfsCollectStates(start_key, submap_radius_);
  if (collected_keys.empty()) return;

  // Assemble world-frame submap
  auto merged = std::make_shared<small_gicp::PointCloud>();
  auto poses_6d = map_manager.getKeyPoses6D();

  for (gtsam::Key k : collected_keys) {
    auto cloud_data = map_manager.getKeyframeData(k, "liso_factor/gicp_cloud");
    if (!cloud_data.has_value()) continue;

    small_gicp::PointCloud::Ptr body_cloud;
    try {
      body_cloud = std::any_cast<small_gicp::PointCloud::Ptr>(cloud_data.value());
    } catch (const std::bad_any_cast&) {
      continue;
    }
    if (!body_cloud || body_cloud->empty()) continue;

    // Get optimized pose for this key
    int idx = map_manager.getCloudIndex(k);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    const auto& pose = poses_6d->points[idx];
    Eigen::Affine3f world_T = poseTypeToAffine3f(pose);
    Eigen::Isometry3d world_T_d;
    world_T_d.matrix() = world_T.matrix().cast<double>();

    // Transform body-frame cloud to world frame
    for (size_t i = 0; i < body_cloud->size(); i++) {
      Eigen::Vector3d p = world_T_d * body_cloud->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) return;

  // Preprocess merged submap: downsample + normals + covariances + KdTree
  auto [submap, submap_tree] = small_gicp::preprocess_points(
      *merged, submap_ds_resolution_, num_neighbors_, num_threads_);

  // Update cached submap under exclusive lock
  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_ = submap;
    cached_submap_tree_ = submap_tree;
  }

  RCLCPP_DEBUG(node_->get_logger(), "[%s] submap rebuilt: %zu states, %zu points",
               name_.c_str(), collected_keys.size(), submap->size());
}

// ---------------------------------------------------------------------------
// bfsCollectStates — BFS over accumulated graph, radial distance bounded
// ---------------------------------------------------------------------------
std::vector<gtsam::Key> LisoFactor::bfsCollectStates(
    gtsam::Key start, double radius) {
  const auto& graph = core_->getAccumulatedGraph();
  const auto& map_manager = core_->getMapManager();
  auto poses_6d = map_manager.getKeyPoses6D();

  // Build adjacency list from the accumulated graph
  std::unordered_map<gtsam::Key, std::vector<gtsam::Key>> adjacency;
  for (size_t i = 0; i < graph.size(); i++) {
    auto factor = graph[i];
    if (!factor) continue;
    auto keys = factor->keys();
    // Only consider factors connecting Pose3-valued keys
    for (size_t a = 0; a < keys.size(); a++) {
      for (size_t b = a + 1; b < keys.size(); b++) {
        adjacency[keys[a]].push_back(keys[b]);
        adjacency[keys[b]].push_back(keys[a]);
      }
    }
  }

  // Get start position for distance check
  int start_idx = map_manager.getCloudIndex(start);
  if (start_idx < 0) return {};
  const auto& start_pose = poses_6d->points[start_idx];
  Eigen::Vector3f start_pos(start_pose.x, start_pose.y, start_pose.z);

  // BFS
  std::vector<gtsam::Key> collected;
  std::unordered_set<gtsam::Key> visited;
  std::queue<gtsam::Key> frontier;

  frontier.push(start);
  visited.insert(start);

  while (!frontier.empty() &&
         static_cast<int>(collected.size()) < max_submap_states_) {
    gtsam::Key current = frontier.front();
    frontier.pop();

    // Check if this key has a pose in MapManager
    int idx = map_manager.getCloudIndex(current);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    // Check radial distance
    const auto& current_pose = poses_6d->points[idx];
    Eigen::Vector3f current_pos(current_pose.x, current_pose.y, current_pose.z);
    float dist = (current_pos - start_pos).norm();
    if (dist > radius) continue;

    collected.push_back(current);

    // Expand neighbors
    auto it = adjacency.find(current);
    if (it == adjacency.end()) continue;
    for (gtsam::Key neighbor : it->second) {
      if (visited.count(neighbor) == 0) {
        visited.insert(neighbor);
        frontier.push(neighbor);
      }
    }
  }

  return collected;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LisoFactor, eidos::FactorPlugin)
