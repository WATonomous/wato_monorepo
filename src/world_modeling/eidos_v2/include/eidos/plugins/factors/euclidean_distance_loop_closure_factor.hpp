#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos {

/**
 * @brief Loop closure via euclidean-distance candidate search + GICP.
 *
 * Latching plugin: triggered when a keyframe-producing plugin creates a
 * new state.  Heavy GICP runs on a background thread; result is delivered
 * on the next latchFactor() call so it always lands in a cycle with a
 * new state (ensuring last_optimized_pose_ / map->odom update).
 *
 * Submaps are assembled in body frame of the center keyframe, and the
 * ISAM2-estimated relative pose is used as GICP init_guess — this avoids
 * the identity-init-guess problem where large rotational drift prevents
 * convergence.
 */
class EuclideanDistanceLoopClosureFactor : public FactorPlugin {
public:
  EuclideanDistanceLoopClosureFactor() = default;
  ~EuclideanDistanceLoopClosureFactor() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  /// Latching plugin: delivers pending loop closure results + dispatches new GICP searches.
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  // BFS over adjacency graph, bounded by radius and max states
  std::vector<gtsam::Key> bfsCollectStates(
      gtsam::Key start, double radius, int max_states);

  // Assemble submap in center_key's body frame from neighbor keys' clouds
  std::pair<small_gicp::PointCloud::Ptr,
            std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
  assembleBodyFrameSubmap(gtsam::Key center_key,
                          const std::vector<gtsam::Key>& keys);

  // Async GICP worker
  void runGICP(gtsam::Key source_key, int source_idx,
               gtsam::Key candidate_key, int candidate_idx);

  // Pending result from background GICP
  struct LoopConstraint {
    gtsam::Key from_key;
    gtsam::Key to_key;
    gtsam::Pose3 relative_pose;
    gtsam::noiseModel::Base::shared_ptr noise;
  };

  std::optional<LoopConstraint> pending_result_;
  mutable std::mutex result_mtx_;

  // Keys already used as loop closure source (avoid re-checking)
  std::set<gtsam::Key> processed_source_keys_;

  // Background GICP thread
  std::atomic<bool> gicp_in_progress_{false};
  std::thread gicp_thread_;
  bool active_ = false;

  // Parameters
  double search_radius_ = 15.0;
  double search_time_diff_ = 30.0;
  int search_num_ = 25;
  double min_inlier_ratio_ = 0.3;
  double submap_leaf_size_ = 0.4;
  double submap_radius_ = 25.0;
  double max_correspondence_distance_ = 2.0;
  int max_iterations_ = 100;
  int num_threads_ = 4;
  int num_neighbors_ = 10;
  std::vector<double> loop_closure_cov_ = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  std::string pointcloud_from_;
};

}  // namespace eidos
