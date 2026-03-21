#include "eidos/plugins/factors/euclidean_distance_loop_closure_factor.hpp"

#include <queue>
#include <unordered_set>

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
  if (gicp_thread_.joinable()) {
    gicp_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void EuclideanDistanceLoopClosureFactor::onInitialize() {
  std::string prefix = name_;

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
  // Legacy parameter — no longer used but declared to avoid YAML errors
  node_->declare_parameter(prefix + ".frequency", 1.0);

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

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (latching, pointcloud_from=%s)",
              name_.c_str(), pointcloud_from_.c_str());
}

void EuclideanDistanceLoopClosureFactor::activate() {
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated (latching mode)", name_.c_str());
}

void EuclideanDistanceLoopClosureFactor::deactivate() {
  active_ = false;
  if (gicp_thread_.joinable()) {
    gicp_thread_.join();
  }
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void EuclideanDistanceLoopClosureFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  std::lock_guard<std::mutex> lock(result_mtx_);
  pending_result_.reset();
  processed_source_keys_.clear();
}

// ---------------------------------------------------------------------------
// FactorPlugin interface — not a state-producing plugin
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3>
EuclideanDistanceLoopClosureFactor::processFrame(double /*timestamp*/) {
  return std::nullopt;
}

StampedFactorResult
EuclideanDistanceLoopClosureFactor::getFactors(gtsam::Key /*key*/) {
  return {};
}

// ---------------------------------------------------------------------------
// latchFactors — called for every new keyframe
//   Phase 1: deliver any completed GICP result
//   Phase 2: KD-tree candidate search → dispatch async GICP
// ---------------------------------------------------------------------------
StampedFactorResult
EuclideanDistanceLoopClosureFactor::latchFactors(gtsam::Key /*key*/, double /*timestamp*/) {
  StampedFactorResult result;
  if (!active_) return result;
  if (core_->getState() != SlamState::TRACKING) return result;

  // Phase 1: deliver pending GICP result
  {
    std::lock_guard<std::mutex> lock(result_mtx_);
    if (pending_result_.has_value()) {
      auto& lc = pending_result_.value();
      auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          lc.from_key, lc.to_key, lc.relative_pose, lc.noise);
      result.factors.push_back(factor);

      gtsam::Symbol from_sym(lc.from_key), to_sym(lc.to_key);
      auto rel_t = lc.relative_pose.translation();
      auto rel_rpy = lc.relative_pose.rotation().rpy();
      RCLCPP_INFO(node_->get_logger(),
          "\033[31m[%s] DELIVERING loop closure: (%c,%lu)->(%c,%lu) "
          "t=(%.2f,%.2f,%.2f) rpy=(%.1f,%.1f,%.1f)\033[0m",
          name_.c_str(), from_sym.chr(), from_sym.index(),
          to_sym.chr(), to_sym.index(),
          rel_t.x(), rel_t.y(), rel_t.z(),
          rel_rpy(0)*180.0/M_PI, rel_rpy(1)*180.0/M_PI, rel_rpy(2)*180.0/M_PI);

      core_->getMapManager().addKeyframeData(
          lc.from_key, name_ + "/loop_target", lc.to_key);

      pending_result_.reset();
    }
  }

  // Phase 2: find new candidates (skip if GICP already running)
  if (gicp_in_progress_.load()) return result;

  const auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();

  if (key_list.size() < 3) return result;

  // Source = latest key with full data in MapManager
  gtsam::Key source_key = key_list.back();
  if (processed_source_keys_.count(source_key)) return result;
  if (map_manager.isPriorMapKey(source_key)) return result;

  int source_idx = map_manager.getCloudIndex(source_key);
  if (source_idx < 0 || source_idx >= static_cast<int>(key_poses_3d->size()))
    return result;

  // KD-tree: spatially close, temporally distant candidates
  auto kdtree = core_->getMapManager().getKdTree();
  if (!kdtree || key_poses_3d->empty()) return result;

  std::vector<int> search_indices;
  std::vector<float> search_distances;
  kdtree->radiusSearch(key_poses_3d->points[source_idx],
                       static_cast<float>(search_radius_),
                       search_indices, search_distances, 0);

  float source_time = key_poses_6d->points[source_idx].time;
  gtsam::Key best_key = 0;
  int best_idx = -1;
  float best_dist = std::numeric_limits<float>::max();

  for (size_t i = 0; i < search_indices.size(); i++) {
    int candidate_idx = search_indices[i];
    if (candidate_idx == source_idx) continue;

    float time_diff =
        std::abs(source_time - key_poses_6d->points[candidate_idx].time);
    if (time_diff < static_cast<float>(search_time_diff_)) continue;

    gtsam::Key candidate_key = map_manager.getKeyFromCloudIndex(candidate_idx);
    if (candidate_key == 0) continue;

    // Skip prior map keys — not in ISAM2
    if (map_manager.isPriorMapKey(candidate_key)) continue;

    if (search_distances[i] < best_dist) {
      best_dist = search_distances[i];
      best_key = candidate_key;
      best_idx = candidate_idx;
    }
  }

  if (best_key == 0) return result;

  // Mark as processed, dispatch async GICP
  processed_source_keys_.insert(source_key);

  if (gicp_thread_.joinable()) {
    gicp_thread_.join();
  }

  gicp_in_progress_ = true;
  gicp_thread_ = std::thread(
      &EuclideanDistanceLoopClosureFactor::runGICP, this,
      source_key, source_idx, best_key, best_idx);

  gtsam::Symbol src_sym(source_key), cand_sym(best_key);
  RCLCPP_INFO(node_->get_logger(),
      "[%s] candidate: (%c,%lu)->(%c,%lu) dist=%.1fm, dispatching GICP",
      name_.c_str(), src_sym.chr(), src_sym.index(),
      cand_sym.chr(), cand_sym.index(), std::sqrt(best_dist));

  return result;
}

// ---------------------------------------------------------------------------
// runGICP — background thread
//   Build body-frame submaps, align with ISAM2 relative pose as init_guess,
//   queue BetweenFactor for delivery on next latchFactors call.
// ---------------------------------------------------------------------------
void EuclideanDistanceLoopClosureFactor::runGICP(
    gtsam::Key source_key, int source_idx,
    gtsam::Key candidate_key, int candidate_idx) {

  auto t0 = std::chrono::steady_clock::now();
  auto ms_since = [&t0]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  };

  // Source: raw body-frame cloud only (independent of ISAM2 — pure sensor data).
  // Using neighbors would make the submap derived from ISAM2 poses, causing
  // GICP to just confirm the current estimate instead of finding drift.
  const auto& map_manager = core_->getMapManager();
  std::string gicp_key = pointcloud_from_ + "/gicp_cloud";

  auto source_cloud_data = map_manager.getKeyframeData(source_key, gicp_key);
  if (!source_cloud_data.has_value()) {
    gicp_in_progress_ = false;
    return;
  }
  small_gicp::PointCloud::Ptr source_body;
  try {
    source_body = std::any_cast<small_gicp::PointCloud::Ptr>(source_cloud_data.value());
  } catch (const std::bad_any_cast&) {
    gicp_in_progress_ = false;
    return;
  }
  if (!source_body || source_body->empty()) {
    gicp_in_progress_ = false;
    return;
  }

  // Preprocess source cloud (normals + covariances + KdTree)
  auto [source_submap, source_tree] = small_gicp::preprocess_points(
      *source_body, submap_leaf_size_, num_neighbors_, num_threads_);
  double t_src = ms_since();

  // Candidate: assembled submap from neighbors (locally accurate, independent
  // of global drift since neighbor relative poses have negligible error)
  auto candidate_keys = bfsCollectStates(candidate_key, submap_radius_, search_num_);
  if (candidate_keys.empty()) {
    gicp_in_progress_ = false;
    return;
  }
  double t_bfs = ms_since();

  auto [candidate_submap, candidate_tree] =
      assembleBodyFrameSubmap(candidate_key, candidate_keys);
  double t_cand = ms_since();

  if (!source_submap || source_submap->empty() ||
      !candidate_submap || candidate_submap->empty()) {
    gicp_in_progress_ = false;
    return;
  }

  // Init guess: ISAM2 relative pose estimate (source_body → candidate_body)
  auto key_poses_6d = map_manager.getKeyPoses6D();

  gtsam::Pose3 T_source =
      poseTypeToGtsamPose3(key_poses_6d->points[source_idx]);
  gtsam::Pose3 T_candidate =
      poseTypeToGtsamPose3(key_poses_6d->points[candidate_idx]);

  // T_candidate^{-1} * T_source maps source_body points → candidate_body
  gtsam::Pose3 relative_estimate = T_candidate.inverse().compose(T_source);
  Eigen::Isometry3d init_guess;
  init_guess.matrix() = relative_estimate.matrix();

  // GICP: align source (source body frame) → candidate (candidate body frame)
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.max_iterations = max_iterations_;
  setting.num_threads = num_threads_;

  auto result = small_gicp::align(
      *candidate_submap, *source_submap, *candidate_tree,
      init_guess, setting);

  double t_gicp = ms_since();

  gtsam::Symbol src_sym(source_key), cand_sym(candidate_key);

  if (!result.converged) {
    RCLCPP_INFO(node_->get_logger(),
        "[%s] GICP did not converge: (%c,%lu)->(%c,%lu) | "
        "scan=%zu pts, cand=%zu states/%zu pts | "
        "src=%.0fms bfs=%.0fms cand=%.0fms gicp=%.0fms total=%.0fms",
        name_.c_str(), src_sym.chr(), src_sym.index(),
        cand_sym.chr(), cand_sym.index(),
        source_submap->size(),
        candidate_keys.size(), candidate_submap->size(),
        t_src, t_bfs - t_src, t_cand - t_bfs, t_gicp - t_cand, t_gicp);
    gicp_in_progress_ = false;
    return;
  }

  // Quality check
  double inlier_ratio =
      static_cast<double>(result.num_inliers) / source_submap->size();
  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_INFO(node_->get_logger(),
        "[%s] rejected: (%c,%lu)->(%c,%lu) inliers=%zu/%zu (%.1f%% < %.1f%%)",
        name_.c_str(), src_sym.chr(), src_sym.index(),
        cand_sym.chr(), cand_sym.index(),
        result.num_inliers, source_submap->size(),
        inlier_ratio * 100.0, min_inlier_ratio_ * 100.0);
    gicp_in_progress_ = false;
    return;
  }

  // BetweenFactor measurement:
  //   T_target_source maps source_body → candidate_body
  //   source.between(candidate) = T_source^{-1} * T_candidate = T_target_source^{-1}
  gtsam::Pose3 T_ts(gtsam::Rot3(result.T_target_source.rotation()),
                     gtsam::Point3(result.T_target_source.translation()));
  gtsam::Pose3 measured = T_ts.inverse();

  auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << loop_closure_cov_[3], loop_closure_cov_[4],
       loop_closure_cov_[5], loop_closure_cov_[0], loop_closure_cov_[1],
       loop_closure_cov_[2]).finished());

  // Log init_guess vs result to see if GICP found a real correction
  gtsam::Pose3 init_pose(gtsam::Rot3(init_guess.rotation()),
                          gtsam::Point3(init_guess.translation()));
  gtsam::Pose3 gicp_correction = init_pose.between(T_ts);
  auto corr_t = gicp_correction.translation();
  double corr_angle = gtsam::Rot3::Logmap(gicp_correction.rotation()).norm();

  auto meas_t = measured.translation();
  auto meas_rpy = measured.rotation().rpy();
  RCLCPP_INFO(node_->get_logger(),
      "\033[31m[%s] LOOP CLOSURE: (%c,%lu)->(%c,%lu) inliers=%zu/%zu (%.1f%%) "
      "measured t=(%.2f,%.2f,%.2f) rpy=(%.1f,%.1f,%.1f) | "
      "gicp_correction: d_pos=%.3fm d_rot=%.2f° | "
      "scan=%zu pts, cand=%zu states/%zu pts | "
      "src=%.0f bfs=%.0f cand=%.0f gicp=%.0fms total=%.0fms\033[0m",
      name_.c_str(), src_sym.chr(), src_sym.index(),
      cand_sym.chr(), cand_sym.index(),
      result.num_inliers, source_submap->size(), inlier_ratio * 100.0,
      meas_t.x(), meas_t.y(), meas_t.z(),
      meas_rpy(0)*180.0/M_PI, meas_rpy(1)*180.0/M_PI, meas_rpy(2)*180.0/M_PI,
      corr_t.norm(), corr_angle * 180.0 / M_PI,
      source_submap->size(),
      candidate_keys.size(), candidate_submap->size(),
      t_src, t_bfs - t_src, t_cand - t_bfs, t_gicp - t_cand, t_gicp);

  {
    std::lock_guard<std::mutex> lock(result_mtx_);
    pending_result_ = LoopConstraint{source_key, candidate_key, measured, noise};
  }

  gicp_in_progress_ = false;
}

// ---------------------------------------------------------------------------
// assembleBodyFrameSubmap — build submap in center_key's body frame
// ---------------------------------------------------------------------------
std::pair<small_gicp::PointCloud::Ptr,
          std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
EuclideanDistanceLoopClosureFactor::assembleBodyFrameSubmap(
    gtsam::Key center_key,
    const std::vector<gtsam::Key>& keys) {

  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  std::string gicp_key = pointcloud_from_ + "/gicp_cloud";

  int center_idx = map_manager.getCloudIndex(center_key);
  if (center_idx < 0 || center_idx >= static_cast<int>(key_poses_6d->size()))
    return {nullptr, nullptr};

  // T_center: world pose of center keyframe
  Eigen::Affine3f center_world_f =
      poseTypeToAffine3f(key_poses_6d->points[center_idx]);
  Eigen::Isometry3d T_center;
  T_center.matrix() = center_world_f.matrix().cast<double>();
  Eigen::Isometry3d T_center_inv = T_center.inverse();

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

    // T_center^{-1} * T_k: transforms k's body-frame points into center's body frame
    Eigen::Affine3f k_world_f =
        poseTypeToAffine3f(key_poses_6d->points[idx]);
    Eigen::Isometry3d T_k;
    T_k.matrix() = k_world_f.matrix().cast<double>();
    Eigen::Isometry3d center_from_k = T_center_inv * T_k;

    for (size_t i = 0; i < body_cloud->size(); i++) {
      Eigen::Vector3d p = center_from_k * body_cloud->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) return {nullptr, nullptr};

  auto [submap, submap_tree] = small_gicp::preprocess_points(
      *merged, submap_leaf_size_, num_neighbors_, num_threads_);

  return {submap, submap_tree};
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

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::EuclideanDistanceLoopClosureFactor,
                       eidos::FactorPlugin)
