#include "eidos/plugins/factors/euclidean_distance_loop_closure_factor.hpp"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pluginlib/class_list_macros.hpp>
#include <gtsam/slam/BetweenFactor.h>

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

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".frequency", 1.0);
  node_->declare_parameter(prefix + ".search_radius", 15.0);
  node_->declare_parameter(prefix + ".search_time_diff", 30.0);
  node_->declare_parameter(prefix + ".search_num", 25);
  node_->declare_parameter(prefix + ".fitness_score", 0.3);
  node_->declare_parameter(prefix + ".mapping_surf_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".pointcloud_from", "lidar_kep_factor");

  // ---- Read parameters ----
  node_->get_parameter(prefix + ".frequency", frequency_);
  node_->get_parameter(prefix + ".search_radius", search_radius_);
  node_->get_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->get_parameter(prefix + ".search_num", search_num_);
  node_->get_parameter(prefix + ".fitness_score", fitness_score_);
  node_->get_parameter(prefix + ".mapping_surf_leaf_size",
                       mapping_surf_leaf_size_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
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
  loop_index_container_.clear();
}

// ---------------------------------------------------------------------------
// processFrame - loop closure does not produce a pose estimate
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3>
EuclideanDistanceLoopClosureFactor::processFrame(double /*timestamp*/) {
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// getFactors - drain the loop constraint queue
// ---------------------------------------------------------------------------
std::vector<gtsam::NonlinearFactor::shared_ptr>
EuclideanDistanceLoopClosureFactor::getFactors(
    int /*state_index*/, const gtsam::Pose3& /*state_pose*/,
    double /*timestamp*/) {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  std::lock_guard<std::mutex> lock(loop_queue_mtx_);
  for (auto& lc : loop_queue_) {
    auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        lc.from_index, lc.to_index, lc.relative_pose, lc.noise);
    factors.push_back(factor);
    RCLCPP_INFO(node_->get_logger(),
                "[%s] Adding loop closure factor: %d -> %d", name_.c_str(),
                lc.from_index, lc.to_index);
  }
  loop_queue_.clear();

  return factors;
}

// ---------------------------------------------------------------------------
// Background thread
// ---------------------------------------------------------------------------
void EuclideanDistanceLoopClosureFactor::loopClosureThread() {
  if (frequency_ <= 0.0f) return;

  rclcpp::Rate rate(static_cast<double>(frequency_));
  while (running_ && rclcpp::ok()) {
    performLoopClosure();
    rate.sleep();
  }
}

void EuclideanDistanceLoopClosureFactor::performLoopClosure() {
  // Need enough keyframes
  auto key_poses_3d = core_->getMapManager().getKeyPoses3D();
  auto key_poses_6d = core_->getMapManager().getKeyPoses6D();
  if (key_poses_3d->size() < 3) return;

  // 1. Find loop closure candidates
  int latest_id = -1, closest_id = -1;
  if (!detectLoopClosureDistance(&latest_id, &closest_id)) return;

  // 2. Build submap around the candidate
  auto near_keyframes = pcl::make_shared<pcl::PointCloud<PointType>>();
  loopFindNearKeyframes(near_keyframes, closest_id, search_num_);
  if (near_keyframes->empty()) return;

  // 3. Build current scan from the latest keyframe
  auto current_scan = pcl::make_shared<pcl::PointCloud<PointType>>();
  loopFindNearKeyframes(current_scan, latest_id, 0);
  if (current_scan->empty()) return;

  // 4. ICP alignment
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(search_radius_ * 2);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(current_scan);
  icp.setInputTarget(near_keyframes);

  auto result_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  icp.align(*result_cloud);

  if (!icp.hasConverged() || icp.getFitnessScore() > fitness_score_) return;

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Loop closure detected: %d -> %d (fitness: %.3f)",
              name_.c_str(), latest_id, closest_id,
              icp.getFitnessScore());

  // 5. Get the relative pose from ICP
  Eigen::Affine3f correction_affine;
  correction_affine = icp.getFinalTransformation();

  // Compute relative pose between the two keyframes
  auto& pose_from = key_poses_6d->points[latest_id];
  auto& pose_to = key_poses_6d->points[closest_id];
  gtsam::Pose3 pose_from_g = poseTypeToGtsamPose3(pose_from);
  gtsam::Pose3 pose_to_g = poseTypeToGtsamPose3(pose_to);

  // Apply ICP correction to the latest pose
  Eigen::Affine3f latest_affine = poseTypeToAffine3f(pose_from);
  Eigen::Affine3f corrected_affine = correction_affine * latest_affine;
  float cx, cy, cz, croll, cpitch, cyaw;
  pcl::getTranslationAndEulerAngles(corrected_affine, cx, cy, cz, croll,
                                    cpitch, cyaw);
  gtsam::Pose3 corrected_pose = gtsam::Pose3(
      gtsam::Rot3::RzRyRx(croll, cpitch, cyaw),
      gtsam::Point3(cx, cy, cz));

  gtsam::Pose3 relative_pose = corrected_pose.between(pose_to_g);

  // Noise based on ICP fitness score with robust Cauchy kernel
  float noise_score = static_cast<float>(icp.getFitnessScore());
  gtsam::Vector6 noise_vec;
  noise_vec << noise_score, noise_score, noise_score, noise_score, noise_score,
      noise_score;
  auto robust_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(noise_vec));

  // 6. Queue the loop constraint
  LoopConstraint lc;
  lc.from_index = latest_id;
  lc.to_index = closest_id;
  lc.relative_pose = relative_pose;
  lc.noise = robust_noise;

  {
    std::lock_guard<std::mutex> lock(loop_queue_mtx_);
    loop_queue_.push_back(lc);
    // Mark this pair so we don't detect it again
    loop_index_container_[latest_id] = closest_id;
  }

  // Store loop target for visualization
  core_->getMapManager().addKeyframeData(latest_id, name_ + "/loop_target", closest_id);
}

bool EuclideanDistanceLoopClosureFactor::detectLoopClosureDistance(
    int* latest_id, int* closest_id) {
  auto key_poses_3d = core_->getMapManager().getKeyPoses3D();
  auto key_poses_6d = core_->getMapManager().getKeyPoses6D();
  int num_poses = static_cast<int>(key_poses_3d->size());
  if (num_poses < 3) return false;

  *latest_id = num_poses - 1;

  // Snapshot loop_index_container_ under lock to avoid data race with reset()
  std::map<int, int> loop_index_snapshot;
  {
    std::lock_guard<std::mutex> lock(loop_queue_mtx_);
    loop_index_snapshot = loop_index_container_;
  }

  // Check if we already found a loop for this keyframe
  if (loop_index_snapshot.find(*latest_id) != loop_index_snapshot.end())
    return false;

  // KD-tree search for nearby keyframes
  auto kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  kdtree->setInputCloud(key_poses_3d);

  std::vector<int> search_indices;
  std::vector<float> search_distances;
  kdtree->radiusSearch(key_poses_3d->points[*latest_id], search_radius_,
                       search_indices, search_distances, 0);

  // Find the closest keyframe that is old enough (time diff check)
  float latest_time = key_poses_6d->points[*latest_id].time;
  int best_id = -1;
  float best_dist = std::numeric_limits<float>::max();

  for (size_t i = 0; i < search_indices.size(); ++i) {
    int candidate = search_indices[i];
    if (candidate == *latest_id) continue;

    float time_diff =
        std::abs(latest_time - key_poses_6d->points[candidate].time);
    if (time_diff < search_time_diff_) continue;

    // Check if already used in a loop closure
    if (loop_index_snapshot.find(candidate) != loop_index_snapshot.end())
      continue;

    if (search_distances[i] < best_dist) {
      best_dist = search_distances[i];
      best_id = candidate;
    }
  }

  if (best_id < 0) return false;

  *closest_id = best_id;
  return true;
}

void EuclideanDistanceLoopClosureFactor::loopFindNearKeyframes(
    pcl::PointCloud<PointType>::Ptr& near_keyframes, int key,
    int search_num) {
  near_keyframes->clear();

  const auto& map_manager = core_->getMapManager();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_poses = static_cast<int>(key_poses_3d->size());

  if (key < 0 || key >= num_poses) return;

  // Collect surrounding keyframe clouds from the producer plugin
  for (int i = -search_num; i <= search_num; ++i) {
    int idx = key + i;
    if (idx < 0 || idx >= num_poses) continue;

    // Get all point cloud data from the producer plugin
    auto plugin_data = map_manager.getKeyframeDataForPlugin(idx, pointcloud_from_);
    if (plugin_data.empty()) continue;

    auto& pose = key_poses_6d->points[idx];
    Eigen::Affine3f t = poseTypeToAffine3f(pose);

    for (const auto& [data_key, data] : plugin_data) {
      try {
        auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        if (cloud && !cloud->empty()) {
          pcl::PointCloud<PointType> transformed;
          pcl::transformPointCloud(*cloud, transformed, t);
          *near_keyframes += transformed;
        }
      } catch (const std::bad_any_cast&) {
        // Not a point cloud type, skip
      }
    }
  }

  if (near_keyframes->empty()) return;

  // Downsample
  pcl::VoxelGrid<PointType> downsample;
  downsample.setLeafSize(mapping_surf_leaf_size_, mapping_surf_leaf_size_,
                         mapping_surf_leaf_size_);
  downsample.setInputCloud(near_keyframes);
  downsample.filter(*near_keyframes);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::EuclideanDistanceLoopClosureFactor,
                       eidos::FactorPlugin)
