#include "eidos/plugins/factors/euclidean_distance_loop_closure_factor.hpp"

#include <queue>
#include <unordered_set>

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
  node_->declare_parameter(prefix + ".submap_radius", submap_radius_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".loop_closure_cov", loop_closure_cov_);
  node_->declare_parameter(prefix + ".pointcloud_from", std::string(""));

  node_->get_parameter(prefix + ".frequency", frequency_);
  node_->get_parameter(prefix + ".search_radius", search_radius_);
  node_->get_parameter(prefix + ".search_time_diff", search_time_diff_);
  node_->get_parameter(prefix + ".search_num", search_num_);
  node_->get_parameter(prefix + ".min_inlier_ratio", min_inlier_ratio_);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->get_parameter(prefix + ".submap_radius", submap_radius_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_iterations", max_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".loop_closure_cov", loop_closure_cov_);
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

  auto t0 = std::chrono::steady_clock::now();
  auto ms_since = [&t0]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  };

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

  double t_search = ms_since();

  // Build two submaps using depth-limited BFS on the adjacency graph:
  //   current_submap: neighborhood around latest_key (where we are now)
  //   historical_submap: neighborhood around best_key (where we were before)
  auto current_keys = bfsCollectStates(latest_key, submap_radius_, search_num_);
  auto historical_keys = bfsCollectStates(best_key, submap_radius_, search_num_);

  if (current_keys.empty() || historical_keys.empty()) return;

  double t_bfs = ms_since();

  auto [current_submap, current_tree] = assembleSubmap(current_keys);
  double t_assemble_cur = ms_since();

  auto [historical_submap, historical_tree] = assembleSubmap(historical_keys);
  double t_assemble_hist = ms_since();

  if (!current_submap || current_submap->empty() ||
      !historical_submap || historical_submap->empty()) return;

  // Initial guess: identity, since both submaps are already in world frame
  // and the two neighborhoods should overlap spatially at the loop closure point
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();

  // Register: align current submap to historical submap
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.max_iterations = max_iterations_;
  setting.num_threads = num_threads_;

  auto result = small_gicp::align(
      *historical_submap, *current_submap, *historical_tree,
      init_guess, setting);

  double t_gicp = ms_since();

  if (!result.converged) {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] GICP did not converge | cur=%zu states/%zu pts, hist=%zu states/%zu pts | "
                "search=%.0fms bfs=%.0fms assemble_cur=%.0fms assemble_hist=%.0fms gicp=%.0fms total=%.0fms",
                name_.c_str(),
                current_keys.size(), current_submap->size(),
                historical_keys.size(), historical_submap->size(),
                t_search, t_bfs - t_search,
                t_assemble_cur - t_bfs, t_assemble_hist - t_assemble_cur,
                t_gicp - t_assemble_hist, t_gicp);
    return;
  }

  // Quality check: inlier ratio
  double inlier_ratio =
      static_cast<double>(result.num_inliers) / current_submap->size();
  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] rejected: inliers=%zu/%zu (%.1f%% < %.1f%%)",
                name_.c_str(), result.num_inliers, current_submap->size(),
                inlier_ratio * 100.0, min_inlier_ratio_ * 100.0);
    return;
  }

  // T_target_source is the correction that aligns the current submap onto
  // the historical submap.  Both submaps are in world frame, so this
  // correction tells us how the current neighborhood should shift.
  //
  // Compute the BetweenFactor relative pose:
  //   T_latest (optimized world pose of latest_key)
  //   T_candidate (optimized world pose of best_key)
  //   T_correction = result.T_target_source  (maps current -> historical frame)
  //
  //   corrected_latest = T_correction * T_latest
  //   relative_pose = corrected_latest^{-1} * T_candidate
  //     = (T_correction * T_latest).between(T_candidate)
  gtsam::Pose3 T_latest =
      poseTypeToGtsamPose3(key_poses_6d->points[latest_idx]);
  gtsam::Pose3 T_candidate =
      poseTypeToGtsamPose3(key_poses_6d->points[best_idx]);

  gtsam::Pose3 T_correction(
      gtsam::Rot3(result.T_target_source.rotation()),
      gtsam::Point3(result.T_target_source.translation()));

  gtsam::Pose3 corrected_latest = T_correction.compose(T_latest);
  gtsam::Pose3 relative_pose = corrected_latest.between(T_candidate);

  // Noise model: config is [x,y,z,roll,pitch,yaw], GTSAM expects [rot,rot,rot,trans,trans,trans]
  // No robust kernel — the inlier ratio check already filters bad matches,
  // and the Cauchy kernel attenuates the correction for large residuals,
  // making loop closures unable to overcome the chain of tight LISO factors.
  auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << loop_closure_cov_[3], loop_closure_cov_[4], loop_closure_cov_[5],
       loop_closure_cov_[0], loop_closure_cov_[1], loop_closure_cov_[2]).finished());

  RCLCPP_INFO(node_->get_logger(),
              "\033[31m[%s] LOOP CLOSURE: inliers=%zu/%zu (%.1f%%) | "
              "cur=%zu states/%zu pts, hist=%zu states/%zu pts | "
              "search=%.0fms bfs=%.0fms assemble_cur=%.0fms assemble_hist=%.0fms gicp=%.0fms total=%.0fms\033[0m",
              name_.c_str(), result.num_inliers, current_submap->size(),
              inlier_ratio * 100.0,
              current_keys.size(), current_submap->size(),
              historical_keys.size(), historical_submap->size(),
              t_search, t_bfs - t_search,
              t_assemble_cur - t_bfs, t_assemble_hist - t_assemble_cur,
              t_gicp - t_assemble_hist, t_gicp);

  // Queue constraint
  {
    std::lock_guard<std::mutex> lock(loop_queue_mtx_);
    loop_queue_.push_back(
        {latest_key, best_key, relative_pose, noise});
    processed_source_keys_.insert(latest_key);
  }

  // Store loop target for visualization
  core_->getMapManager().addKeyframeData(
      latest_key, name_ + "/loop_target", best_key);
}

// ---------------------------------------------------------------------------
// bfsCollectStates — BFS over adjacency graph, radial distance bounded
// ---------------------------------------------------------------------------
std::vector<gtsam::Key> EuclideanDistanceLoopClosureFactor::bfsCollectStates(
    gtsam::Key start, double radius, int max_states) {
  const auto& map_manager = core_->getMapManager();
  auto poses_6d = map_manager.getKeyPoses6D();
  const auto& adjacency = map_manager.getAdjacency();

  int start_idx = map_manager.getCloudIndex(start);
  if (start_idx < 0) return {};
  const auto& start_pose = poses_6d->points[start_idx];
  Eigen::Vector3f start_pos(start_pose.x, start_pose.y, start_pose.z);

  std::vector<gtsam::Key> collected;
  std::unordered_set<gtsam::Key> visited;
  std::queue<gtsam::Key> frontier;

  frontier.push(start);
  visited.insert(start);

  while (!frontier.empty() &&
         static_cast<int>(collected.size()) < max_states) {
    gtsam::Key current = frontier.front();
    frontier.pop();

    int idx = map_manager.getCloudIndex(current);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    const auto& current_pose = poses_6d->points[idx];
    Eigen::Vector3f current_pos(current_pose.x, current_pose.y, current_pose.z);
    float dist = (current_pos - start_pos).norm();
    if (dist > radius) continue;

    collected.push_back(current);

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

// ---------------------------------------------------------------------------
// assembleSubmap — transform keyframe clouds to world frame, preprocess
// ---------------------------------------------------------------------------
std::pair<small_gicp::PointCloud::Ptr,
          std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
EuclideanDistanceLoopClosureFactor::assembleSubmap(
    const std::vector<gtsam::Key>& keys) {
  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  std::string gicp_key = pointcloud_from_ + "/gicp_cloud";

  auto merged = std::make_shared<small_gicp::PointCloud>();

  for (gtsam::Key k : keys) {
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

  auto [submap, submap_tree] = small_gicp::preprocess_points(
      *merged, submap_leaf_size_, num_neighbors_, num_threads_);

  return {submap, submap_tree};
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::EuclideanDistanceLoopClosureFactor,
                       eidos::FactorPlugin)
