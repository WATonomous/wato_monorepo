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
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief Euclidean distance-based loop closure factor plugin.
 *
 * Runs a background thread that searches for loop closure candidates
 * using KD-tree radius search and validates them using small_gicp GICP
 * between two submaps (current neighborhood vs historical neighborhood).
 */
class EuclideanDistanceLoopClosureFactor : public FactorPlugin {
public:
  EuclideanDistanceLoopClosureFactor() = default;
  ~EuclideanDistanceLoopClosureFactor() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  StampedFactorResult getFactors(gtsam::Key key) override;
  bool hasData() const override;

private:
  void loopClosureThread();
  void performLoopClosure();

  // BFS over adjacency graph, bounded by radius and max states
  std::vector<gtsam::Key> bfsCollectStates(
      gtsam::Key start, double radius, int max_states);

  // Assemble a world-frame submap from the given keyframe keys
  std::pair<small_gicp::PointCloud::Ptr,
            std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
  assembleSubmap(const std::vector<gtsam::Key>& keys);

  struct LoopConstraint {
    gtsam::Key from_key;
    gtsam::Key to_key;
    gtsam::Pose3 relative_pose;
    gtsam::noiseModel::Base::shared_ptr noise;
  };

  std::vector<LoopConstraint> loop_queue_;
  mutable std::mutex loop_queue_mtx_;

  // Source keys that have already been used in a loop closure
  std::set<gtsam::Key> processed_source_keys_;

  std::thread loop_thread_;
  std::atomic<bool> running_{false};
  bool active_ = false;

  // Parameters
  double frequency_ = 1.0;
  double search_radius_ = 15.0;         // meters, radius to find loop candidates
  double search_time_diff_ = 30.0;      // seconds, minimum temporal gap
  int search_num_ = 25;                 // max states per submap (BFS depth limit)
  double min_inlier_ratio_ = 0.3;       // minimum inlier fraction to accept (0-1)
  double submap_leaf_size_ = 0.4;        // voxel size for submap downsampling
  double submap_radius_ = 25.0;          // meters, BFS radius for submap assembly
  double max_correspondence_distance_ = 2.0;
  int max_iterations_ = 100;
  int num_threads_ = 4;
  int num_neighbors_ = 10;              // neighbors for normal/covariance estimation
  std::vector<double> loop_closure_cov_ = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};  // [x,y,z,roll,pitch,yaw] variance
  std::string pointcloud_from_;
};

}  // namespace eidos
