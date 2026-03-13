#include "eidos/plugins/factors/euclidean_distance_loop_closure_factor.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
EuclideanDistanceLoopClosureFactor::~EuclideanDistanceLoopClosureFactor() {
  running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void EuclideanDistanceLoopClosureFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".frequency", frequency_);
  node_->declare_parameter(prefix + ".search_radius", search_radius_);
  node_->declare_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->declare_parameter(prefix + ".search_num", search_num_);
  node_->declare_parameter(prefix + ".min_inlier_ratio", min_inlier_ratio_);
  node_->declare_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".loop_closure_noise", loop_closure_noise_);
  node_->declare_parameter(prefix + ".pointcloud_from", std::string(""));

  node_->get_parameter(prefix + ".frequency", frequency_);
  node_->get_parameter(prefix + ".search_radius", search_radius_);
  node_->get_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->get_parameter(prefix + ".search_num", search_num_);
  node_->get_parameter(prefix + ".min_inlier_ratio", min_inlier_ratio_);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_iterations", max_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".loop_closure_noise", loop_closure_noise_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (pointcloud_from=%s)",
              name_.c_str(), pointcloud_from_.c_str());
}

void EuclideanDistanceLoopClosureFactor::activate() {
  active_ = true;
  running_ = true;
  loop_thread_ = std::thread(
      &EuclideanDistanceLoopClosureFactor::loopClosureThread, this);
  RCLCPP_INFO(node_->get_logger(),
              "[%s] activated (background thread started)", name_.c_str());
}

void EuclideanDistanceLoopClosureFactor::deactivate() {
  active_ = false;
  running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  RCLCPP_INFO(node_->get_logger(),
              "[%s] deactivated (background thread stopped)", name_.c_str());
}

void EuclideanDistanceLoopClosureFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  std::lock_guard<std::mutex> lock(loop_queue_mtx_);
  loop_queue_.clear();
  processed_source_keys_.clear();
}

// ---------------------------------------------------------------------------
// FactorPlugin interface
// ---------------------------------------------------------------------------
bool EuclideanDistanceLoopClosureFactor::hasData() const {
  std::lock_guard<std::mutex> lock(loop_queue_mtx_);
  return !loop_queue_.empty();
}

std::optional<gtsam::Pose3>
EuclideanDistanceLoopClosureFactor::processFrame(double /*timestamp*/) {
  return std::nullopt;
}

StampedFactorResult
EuclideanDistanceLoopClosureFactor::getFactors(gtsam::Key /*key*/) {
  StampedFactorResult result;

  std::lock_guard<std::mutex> lock(loop_queue_mtx_);
  for (auto& lc : loop_queue_) {
    auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        lc.from_key, lc.to_key, lc.relative_pose, lc.noise);
    result.factors.push_back(factor);

    gtsam::Symbol from_sym(lc.from_key), to_sym(lc.to_key);
    RCLCPP_INFO(node_->get_logger(),
                "[%s] loop closure factor: (%c,%lu) -> (%c,%lu)",
                name_.c_str(),
                from_sym.chr(), from_sym.index(),
                to_sym.chr(), to_sym.index());
  }
  loop_queue_.clear();

  return result;
}

// ---------------------------------------------------------------------------
// Background thread
// ---------------------------------------------------------------------------
void EuclideanDistanceLoopClosureFactor::loopClosureThread() {
  if (frequency_ <= 0.0) return;

  rclcpp::Rate rate(frequency_);
  while (running_ && rclcpp::ok()) {
    performLoopClosure();
    rate.sleep();
  }
}

void EuclideanDistanceLoopClosureFactor::performLoopClosure() {
  if (core_->getState() != SlamState::TRACKING) return;

  const auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();

  if (key_list.size() < 3) return;

  // Latest keyframe
  gtsam::Key latest_key = key_list.back();

  // Skip if already processed
  {
    std::lock_guard<std::mutex> lock(loop_queue_mtx_);
    if (processed_source_keys_.count(latest_key)) return;
  }

  int latest_idx = map_manager.getCloudIndex(latest_key);
  if (latest_idx < 0 || latest_idx >= static_cast<int>(key_poses_3d->size()))
    return;

  // KD-tree search for nearby keyframes within search_radius_
  auto kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  kdtree->setInputCloud(key_poses_3d);

  std::vector<int> search_indices;
  std::vector<float> search_distances;
  kdtree->radiusSearch(key_poses_3d->points[latest_idx],
                       static_cast<float>(search_radius_),
                       search_indices, search_distances, 0);

  // Find best candidate: spatially close but temporally distant
  float latest_time = key_poses_6d->points[latest_idx].time;
  gtsam::Key best_key = 0;
  int best_idx = -1;
  float best_dist = std::numeric_limits<float>::max();

  for (size_t i = 0; i < search_indices.size(); i++) {
    int candidate_idx = search_indices[i];
    if (candidate_idx == latest_idx) continue;

    float time_diff =
        std::abs(latest_time - key_poses_6d->points[candidate_idx].time);
    if (time_diff < static_cast<float>(search_time_diff_)) continue;

    gtsam::Key candidate_key = map_manager.getKeyFromCloudIndex(candidate_idx);
    if (candidate_key == 0) continue;

    if (search_distances[i] < best_dist) {
      best_dist = search_distances[i];
      best_key = candidate_key;
      best_idx = candidate_idx;
    }
  }

  if (best_key == 0) return;

  // Assemble candidate submap (world frame, preprocessed)
  auto [submap, submap_tree] = assembleSubmap(best_key, search_num_);
  if (!submap || submap->empty()) return;

  // Get current scan in body frame from MapManager
  std::string gicp_key = pointcloud_from_ + "/gicp_cloud";
  auto scan_data = map_manager.getKeyframeData(latest_key, gicp_key);
  if (!scan_data.has_value()) return;

  small_gicp::PointCloud::Ptr current_scan;
  try {
    current_scan =
        std::any_cast<small_gicp::PointCloud::Ptr>(scan_data.value());
  } catch (const std::bad_any_cast&) {
    return;
  }
  if (!current_scan || current_scan->empty()) return;

  // Initial guess: latest keyframe's optimized world-frame pose
  // (transforms body-frame scan into world-frame submap)
  Eigen::Affine3f latest_affine =
      poseTypeToAffine3f(key_poses_6d->points[latest_idx]);
  Eigen::Isometry3d init_guess;
  init_guess.matrix() = latest_affine.matrix().cast<double>();

  // Register: align body-frame scan to world-frame submap using GICP
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.max_iterations = max_iterations_;
  setting.num_threads = num_threads_;

  auto result = small_gicp::align(
      *submap, *current_scan, *submap_tree, init_guess, setting);

  if (!result.converged) return;

  // Quality check: inlier ratio
  double inlier_ratio =
      static_cast<double>(result.num_inliers) / current_scan->size();
  if (inlier_ratio < min_inlier_ratio_) return;

  // T_target_source = ICP-corrected world-frame pose of the current scan
  gtsam::Pose3 corrected_pose(
      gtsam::Rot3(result.T_target_source.rotation()),
      gtsam::Point3(result.T_target_source.translation()));

  gtsam::Pose3 candidate_pose =
      poseTypeToGtsamPose3(key_poses_6d->points[best_idx]);
  gtsam::Pose3 relative_pose = corrected_pose.between(candidate_pose);

  // Noise model: fixed variance per DOF with robust Cauchy kernel
  gtsam::Vector6 noise_vec =
      gtsam::Vector6::Constant(loop_closure_noise_);
  auto robust_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(noise_vec));

  RCLCPP_INFO(node_->get_logger(),
              "\033[31m[%s] loop closure: inliers=%zu/%zu (%.1f%%)\033[0m",
              name_.c_str(), result.num_inliers, current_scan->size(),
              inlier_ratio * 100.0);

  // Queue constraint
  {
    std::lock_guard<std::mutex> lock(loop_queue_mtx_);
    loop_queue_.push_back(
        {latest_key, best_key, relative_pose, robust_noise});
    processed_source_keys_.insert(latest_key);
  }

  // Store loop target for visualization
  core_->getMapManager().addKeyframeData(
      latest_key, name_ + "/loop_target", best_key);
}

// ---------------------------------------------------------------------------
// assembleSubmap — KNN keyframes, transform to world frame, preprocess
// ---------------------------------------------------------------------------
std::pair<small_gicp::PointCloud::Ptr,
          std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
EuclideanDistanceLoopClosureFactor::assembleSubmap(
    gtsam::Key center_key, int num_neighbors) {
  const auto& map_manager = core_->getMapManager();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();

  int center_idx = map_manager.getCloudIndex(center_key);
  if (center_idx < 0 ||
      center_idx >= static_cast<int>(key_poses_3d->size())) {
    return {nullptr, nullptr};
  }

  // Find neighbor keyframes
  std::vector<gtsam::Key> neighbor_keys;
  if (num_neighbors <= 0) {
    // Single keyframe
    neighbor_keys.push_back(center_key);
  } else {
    // K-nearest neighbors
    auto kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
    kdtree->setInputCloud(key_poses_3d);

    std::vector<int> indices;
    std::vector<float> distances;
    kdtree->nearestKSearch(key_poses_3d->points[center_idx],
                           num_neighbors + 1, indices, distances);

    for (int idx : indices) {
      gtsam::Key k = map_manager.getKeyFromCloudIndex(idx);
      if (k != 0) neighbor_keys.push_back(k);
    }
  }

  // Assemble world-frame point cloud from body-frame keyframe clouds
  std::string gicp_key = pointcloud_from_ + "/gicp_cloud";
  auto merged = std::make_shared<small_gicp::PointCloud>();

  for (gtsam::Key k : neighbor_keys) {
    auto cloud_data = map_manager.getKeyframeData(k, gicp_key);
    if (!cloud_data.has_value()) continue;

    small_gicp::PointCloud::Ptr body_cloud;
    try {
      body_cloud =
          std::any_cast<small_gicp::PointCloud::Ptr>(cloud_data.value());
    } catch (const std::bad_any_cast&) {
      continue;
    }
    if (!body_cloud || body_cloud->empty()) continue;

    int idx = map_manager.getCloudIndex(k);
    if (idx < 0 || idx >= static_cast<int>(key_poses_6d->size())) continue;

    Eigen::Affine3f world_T = poseTypeToAffine3f(key_poses_6d->points[idx]);
    Eigen::Isometry3d world_T_d;
    world_T_d.matrix() = world_T.matrix().cast<double>();

    for (size_t i = 0; i < body_cloud->size(); i++) {
      Eigen::Vector3d p = world_T_d * body_cloud->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) return {nullptr, nullptr};

  // Preprocess: downsample + normals + covariances + KdTree
  auto [submap, submap_tree] = small_gicp::preprocess_points(
      *merged, submap_leaf_size_, num_neighbors_, num_threads_);

  return {submap, submap_tree};
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::EuclideanDistanceLoopClosureFactor,
                       eidos::FactorPlugin)
