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
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <gtsam/inference/Key.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/utils/bnb_search.hpp"
#include "eidos/utils/types.hpp"
#include "eidos/utils/voxel_pyramid.hpp"

namespace eidos
{

// GPS-free global relocalization by branch-and-bound voxel search against a prior map. Builds a
// multi-resolution voxel pyramid from the prior map's keyframe clouds, runs a best-first
// branch-and-bound search over 4-DOF (x, y, z, yaw) with a small roll/pitch grid around the IMU
// gravity estimate, then refines the top hypotheses with small_gicp GICP for a full 6-DOF pose.
// Build and search run on a background worker thread; tryRelocalize() never blocks the SLAM
// loop -- it only launches or polls the worker.
class BnbVoxelRelocalization : public RelocalizationPlugin
{
public:
  BnbVoxelRelocalization() = default;
  ~BnbVoxelRelocalization() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  // Never does heavy work itself: launches the worker thread on first call, then polls it.
  std::optional<RelocalizationResult> tryRelocalize(double timestamp) override;

private:
  // Larger of num_threads_ (shared config cap) and the hardware thread count -- capping search
  // loops at a configured GICP thread count left half a 32-core box idle during search.
  int searchThreads() const;

  bool inHeightBand(double z) const
  {
    if (min_height_ > 0.0 && z < min_height_) return false;
    if (max_height_ > 0.0 && z > max_height_) return false;
    return true;
  }

  // One cached prior-map keyframe used for corridor roots and submap assembly.
  struct TrajectoryEntry
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    int cloud_index = -1;
    gtsam::Key key = 0;
    double yaw = 0.0;  // Recorded map-frame heading; seeds root_headings_ for the fine prefilter.
  };

  // A branch-and-bound hypothesis paired with the roll/pitch offset that produced it.
  struct ScoredHypothesis
  {
    eidos::reloc::Hypothesis hyp;
    double dr = 0.0;
    double dp = 0.0;
  };

  // Best level-0 score (and argmax pose) for one root cell from the fine prefilter scan. Same
  // order/index as `roots_`.
  struct FineRootScore
  {
    int score = -1;  // -1 if unscored (e.g. stop_requested_ fired mid-scan).
    std::size_t idx = 0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    double yaw = 0.0;
  };

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Linear scan over the cached trajectory -- never MapManager::getKdTree(), which is non-const
  // and unsafe off the SLAM thread.
  std::shared_ptr<small_gicp::PointCloud> assembleSubmap(const Eigen::Vector3d & centre, double radius) const;

  // Re-runs branch-and-bound against the CURRENT scan, restricted to a corridor of roots within
  // reanchor_search_radius_ of `locked`, and refines with the same GICP gates the lock passed.
  // Needed because a search can take tens of seconds, during which the vehicle drives on and no
  // odometry is published while RELOCALIZING (both odometry producers gate on TRACKING) -- so the
  // prior map's own trajectory plus a fresh scan is what's available to correct the stale pose.
  bool reanchorToCurrent(const gtsam::Pose3 & locked, gtsam::Pose3 & out);

  void workerMain();

  // Rasterizes prior-map keyframe clouds into the pyramid and builds corridor roots. Runs once.
  bool buildPyramid();

  // Tries the PCL-typed cloud retrieval, then the small_gicp-typed one, so either source works
  // regardless of the configured key's naming. Also raycasts free space when use_free_space_ is
  // set, in the same pass as occupancy insertion (VoxelPyramid::insertRay() isn't thread-safe, so
  // a second pass would double the iteration cost for nothing).
  bool insertKeyframeCloud(
    gtsam::Key key, const std::string & data_key, const Eigen::Isometry3d & world_t,
    const Eigen::Vector3d & sensor_offset_body);

  // Thread-safe insertKeyframeCloud() for buildPyramid()'s parallel path: rasterizes into a
  // caller-owned shard instead of the shared pyramid, and never raycasts free space (see
  // VoxelPyramid::insertIntoShard()). Safe to call concurrently across keyframes as long as each
  // thread owns its shard/out_of_range and no keyframe is shared between threads.
  bool insertKeyframeCloudShard(
    gtsam::Key key, const std::string & data_key, const Eigen::Isometry3d & world_t,
    eidos::reloc::VoxelSet & shard, std::size_t & out_of_range);

  void buildRoots();
  void publishDebugGrid();

  std::vector<ScoredHypothesis> searchPoses(const std::vector<Eigen::Vector3d> & query);

  // Builds a query from the nearest prior-map keyframe's own cloud, de-tilted/filtered/
  // downsampled exactly like the live path -- so the result is, by construction, exactly
  // registered to the map. Used by the debug_self_test_/debug_use_self_query_ diagnostics.
  bool buildSelfQuery(
    const Eigen::Vector3d & near_position, std::vector<Eigen::Vector3d> & query_out,
    Eigen::Vector3d & kf_translation_out, double & kf_yaw_out, int & kf_index_out);

  // Cheaply ranks roots and keeps only the top root_prefilter_keep_ before branchAndBound() --
  // a heuristic shortcut (wrong survivors are still caught by the GICP/uniqueness gates
  // downstream), not a soundness-preserving filter. Two modes via prefilter_fine_: the original
  // coarse-level ranking (kept for A/B testing) saturates almost every root at the same ceiling
  // on this map and was the measured cause of a 281m mislocalization; the fine, heading-
  // constrained level-0 ranking (default) discriminates and is what scoreRootsFine() implements.
  std::vector<eidos::reloc::RootCell> prefilterRoots(
    const std::vector<Eigen::Vector3d> & rotated_query, double dr, double dp);

  // Scores every root's best (position, yaw) at level 0 within a heading-constrained local
  // search inside its cell. Shared verbatim by prefilterRoots() and the TRACE 9 diagnostic so
  // the two can never disagree on ranking.
  std::vector<FineRootScore> scoreRootsFine(const std::vector<Eigen::Vector3d> & sub_query) const;

  std::optional<RelocalizationResult> gicpPolish(const std::vector<ScoredHypothesis> & hypotheses);

  // Releases pyramid_'s memory and logs RSS before/after -- freeing the buffers doesn't by
  // itself guarantee the OS reclaims them, so the log is what makes "memory actually came back"
  // checkable instead of assumed. No-op when the pyramid is already empty.
  void releasePyramidMemory(const char * context);

  // Subscriptions / publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_pub_;

  // Buffered sensor data -- written by callbacks, read by the worker thread.
  small_gicp::PointCloud::Ptr latest_scan_;
  std::vector<Eigen::Vector3d> latest_query_;
  double latest_scan_stamp_ = 0.0;
  std::mutex scan_lock_;

  // Scan the CURRENT search is bound to, latched once at search start (worker-owned, no lock
  // needed) -- the live buffers above keep advancing at sensor rate mid-search, and refining
  // hypotheses against a newer cloud would register the wrong data against the right guess.
  small_gicp::PointCloud::Ptr search_scan_;
  double search_scan_stamp_ = 0.0;

  double latest_imu_roll_ = 0.0;
  double latest_imu_pitch_ = 0.0;
  std::mutex imu_lock_;
  bool has_imu_ = false;

  std::atomic<bool> active_{false};

  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_lidar_tf_ = false;
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;

  std::string base_link_frame_;
  std::string map_frame_;
  std::string lidar_frame_;
  std::string imu_frame_;

  // Launch-or-poll protocol
  std::atomic<bool> search_running_{false};
  std::atomic<bool> result_ready_{false};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::optional<RelocalizationResult> result_;
  std::mutex result_mtx_;

  // Worker-owned state (touched only by the worker thread)
  eidos::reloc::VoxelPyramid pyramid_;
  bool pyramid_built_ = false;
  bool pyramid_failed_ = false;
  pcl::PointCloud<PoseType>::Ptr poses6d_;
  std::vector<TrajectoryEntry> trajectory_;
  std::vector<eidos::reloc::RootCell> roots_;
  std::vector<float> root_headings_;         // Heading of the entry that admitted each root cell.
  std::vector<ScoredHypothesis> prefilter_candidates_;  // Fine prefilter's per-offset argmax poses.

  // Mean normalized score of random corridor poses -- the measured "no real match" floor, since
  // neither scoring mode bottoms out at zero. Used by gicpPolish()'s uniqueness gate.
  double last_chance_floor_ = 0.0;
  bool build_free_space_this_run_ = false;  // Whether free-space raycast actually ran this build.
  eidos::reloc::ScoreMode active_score_mode_ = eidos::reloc::ScoreMode::DistanceField;  // Mode
                                             // actually built with (may differ from score_mode_
                                             // after an overflow fallback).

  // ---- Parameters ----
  std::string pointcloud_from_;
  bool prefer_downsampled_source_ = false;
  double min_voxel_size_ = 1.0;
  int num_levels_ = 4;

  // distance_field (default) vs occupancy (kept for A/B testing). Binary occupancy was measured
  // to carry almost no alignment signal on ring_road.map; the distance field's graded falloff
  // keeps the "near structure" distinction that binary containment throws away.
  eidos::reloc::ScoreMode score_mode_ = eidos::reloc::ScoreMode::DistanceField;
  double df_sigma_ = 1.0;
  int df_truncation_voxels_ = 2;
  std::size_t max_score_voxels_ = 40000000;
  double min_height_ = 0.6;
  double max_height_ = 6.0;
  std::size_t max_voxels_per_level_ = 8000000;
  int target_query_points_ = 400;
  double max_query_range_ = 40.0;
  std::vector<double> debug_probe_pose_;  // Optional [x,y,z,yaw_deg] diagnostic probe.

  // Diagnostic switches, all zero-cost/zero-behaviour-change when false and debug_probe_pose_
  // unset -- see each block's doc comment in the .cpp for procedure/rationale.
  bool debug_band_sweep_ = false;       // Height-band discrimination sweep.
  bool debug_res_sweep_ = false;        // Resolution/scoring-mode/min-observation sweep.
  bool debug_self_test_ = false;        // Scores a keyframe's own cloud at its own pose.
  bool debug_use_self_query_ = false;   // Runs a self-query through the full production path.

  double rp_search_range_ = 0.02;
  int rp_search_steps_ = 1;  // 1 = trust IMU gravity; each extra step multiplies search cost
  double search_corridor_ = 30.0;
  double z_margin_ = 5.0;
  double prune_slack_ = 0.8;
  int max_search_nodes_ = 200000;
  double nms_radius_ = 5.0;
  int root_prefilter_points_ = 128;
  int root_prefilter_keep_ = 256;

  bool prefilter_fine_ = true;  // false = old coarse prefilter (kept for A/B testing).
  bool use_heading_prior_ = true;
  double heading_tolerance_deg_ = 30.0;
  bool allow_reverse_heading_ = true;  // Also scan the tolerance window around heading+180deg.
  double prefilter_xy_step_ = 2.0;
  double prefilter_z_step_ = 4.0;
  double prefilter_yaw_step_deg_ = 10.0;
  double min_match_score_ = 0.45;
  double min_score_ratio_ = 1.20;
  int num_gicp_candidates_ = 5;
  double min_inlier_ratio_ = 0.30;
  double scan_ds_resolution_ = 0.5;
  double submap_radius_ = 40.0;
  double submap_leaf_size_ = 0.4;
  double max_correspondence_distance_ = 2.0;
  int max_icp_iterations_ = 100;
  int num_threads_ = 16;
  int num_neighbors_ = 10;
  bool publish_debug_grid_ = true;
  std::string debug_grid_topic_;

  // ---- Trajectory re-anchor ----
  bool use_trajectory_reanchor_ = true;
  double reanchor_search_radius_ = 250.0;  // Must cover distance travelled during a search.
  int reanchor_max_nodes_ = 40000;
  double reanchor_min_gap_ = 0.5;
  double reanchor_warn_distance_ = 25.0;

  // Root chunks per roll/pitch offset, as a multiple of the search thread count -- the search's
  // only parallelism is over (offset x chunk) tasks, so with one offset (the default) this sets
  // the task count directly; oversubscribing gives schedule(dynamic) something to balance since
  // chunk cost varies widely with how much structure each root's subtree holds.
  int search_task_multiplier_ = 4;

  // ---- Free-space channel (occupancy mode only; ignored under DistanceField) ----
  bool use_free_space_ = false;  // Off by default: didn't help on ring_road.map (the underlying
                                  // occupancy score wasn't discriminative there to begin with).
  int free_rays_per_keyframe_ = 2000;
  double free_max_range_ = 40.0;
  double free_end_margin_ = 1.0;
  bool free_clear_near_occupied_ = true;
  std::size_t max_free_voxels_ = 20000000;
  double free_ray_origin_height_ = 2.0;
  int hit_weight_ = 3;
};

}  // namespace eidos
