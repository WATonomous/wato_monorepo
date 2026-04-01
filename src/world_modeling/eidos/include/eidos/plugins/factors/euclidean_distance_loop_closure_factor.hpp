// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/utils/types.hpp"

namespace eidos
{

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
class EuclideanDistanceLoopClosureFactor : public FactorPlugin
{
public:
  EuclideanDistanceLoopClosureFactor() = default;

  /// @brief Join the background GICP thread if running.
  ~EuclideanDistanceLoopClosureFactor() override;

  /// @brief Declare ROS parameters for search radius, GICP settings, etc.
  void onInitialize() override;

  /// @brief Enable loop closure detection.
  void activate() override;

  /// @brief Disable loop closure detection and join any background thread.
  void deactivate() override;

  /**
   * @brief Deliver pending loop closure results and dispatch new GICP searches.
   *
   * On each call: (1) if a background GICP has completed, returns its
   * BetweenFactor as a loop closure constraint; (2) searches for new
   * candidate keyframes by Euclidean distance and dispatches GICP on a
   * background thread if a candidate is found.
   *
   * @param key GTSAM key of the newly created state.
   * @param timestamp Timestamp of the new state (seconds).
   * @return StampedFactorResult with a loop closure factor if one is ready, or empty.
   */
  StampedFactorResult latchFactor(gtsam::Key key, double timestamp) override;

private:
  /**
   * @brief Collect keys reachable from start via BFS on the adjacency graph.
   * @param start Starting key for BFS.
   * @param radius Maximum Euclidean distance from the start key's position.
   * @param max_states Maximum number of keys to collect.
   * @return Vector of collected keys (up to max_states).
   */
  std::vector<gtsam::Key> bfsCollectStates(gtsam::Key start, double radius, int max_states);

  /**
   * @brief Assemble a submap in the center key's body frame from neighboring keyframe clouds.
   * @param center_key The key whose body frame defines the submap coordinate origin.
   * @param keys Neighboring keys whose point clouds are included.
   * @return Pair of (merged point cloud, KD-tree) in center_key's body frame.
   */
  std::pair<small_gicp::PointCloud::Ptr, std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
  assembleBodyFrameSubmap(gtsam::Key center_key, const std::vector<gtsam::Key> & keys);

  /**
   * @brief Run GICP alignment between source and candidate submaps on a background thread.
   *
   * Assembles body-frame submaps for both keys, aligns them using the ISAM2-estimated
   * relative pose as initial guess, and stores the result in pending_result_.
   *
   * @param source_key GTSAM key of the current (source) keyframe.
   * @param source_idx Integer index of the source keyframe in MapManager.
   * @param candidate_key GTSAM key of the candidate (target) keyframe.
   * @param candidate_idx Integer index of the candidate keyframe in MapManager.
   */
  void runGICP(gtsam::Key source_key, int source_idx, gtsam::Key candidate_key, int candidate_idx);

  // Pending result from background GICP
  struct LoopConstraint
  {
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
  std::string pointcloud_from_;  ///< PCL cloud data key (fallback)
  std::string gicp_pointcloud_from_;  ///< small_gicp cloud data key (preferred, optional)
};

}  // namespace eidos
