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

/**
 * @brief GPS-free global relocalization by branch-and-bound voxel search against a prior map.
 *
 * Builds a multi-resolution sparse-hash voxel pyramid from the prior map's keyframe clouds, runs
 * a best-first branch-and-bound search over 4-DOF (x, y, z, yaw) with a small roll/pitch grid
 * around the IMU gravity estimate, then refines the top hypotheses with small_gicp GICP for a
 * full 6-DOF pose. The pyramid build and search run on a background worker thread; tryRelocalize()
 * never blocks the SLAM loop -- it only launches or polls the worker.
 */
class BnbVoxelRelocalization : public RelocalizationPlugin
{
public:
  BnbVoxelRelocalization() = default;

  /// @brief Stop and join the worker thread, if still running, then release pyramid memory.
  ~BnbVoxelRelocalization() override;

  /// @brief Declare ROS parameters, create LiDAR/IMU subscriptions and the debug grid publisher.
  void onInitialize() override;

  /// @brief Enable sensor subscriptions and allow the worker to (re)launch.
  void activate() override;

  /// @brief Disable relocalization and stop/join the worker thread.
  void deactivate() override;

  /**
   * @brief Launch or poll the background branch-and-bound search worker.
   *
   * This method never performs heavy work itself. On the first call after activation it launches
   * a worker thread that builds the voxel pyramid (once), then repeatedly searches the latest live
   * scan against it. Subsequent calls either return a ready result or return std::nullopt to be
   * retried next tick.
   *
   * @param timestamp Current SLAM cycle timestamp (seconds). Unused -- the worker always operates
   *   on the most recent buffered sensor data.
   * @return RelocalizationResult once the worker locks a pose, std::nullopt otherwise.
   */
  std::optional<RelocalizationResult> tryRelocalize(double timestamp) override;

private:
  /// @brief Whether a body-frame height lies inside the configured matching band.
  /// @param z Body-frame z in metres (base frame sits at ground level).
  /// @return True if the point should be used for matching.
  bool inHeightBand(double z) const
  {
    if (min_height_ > 0.0 && z < min_height_) return false;
    if (max_height_ > 0.0 && z > max_height_) return false;
    return true;
  }

  /// @brief One cached prior-map keyframe used for corridor roots and submap assembly.
  struct TrajectoryEntry
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< Map-frame keyframe position.
    int cloud_index = -1;  ///< Index into the cached 6-DOF pose cloud.
    gtsam::Key key = 0;  ///< GTSAM key, for MapManager::retrieve() calls.
    /// Map-frame heading (yaw, rad) of this keyframe's recorded pose. Populated in buildPyramid()
    /// via `gtsam::Rot3(T.rotation()).yaw()` -- the same `Rz(yaw) * rotYX(pitch, roll)` convention
    /// verified elsewhere in this file (see buildSelfQuery()'s numerical convention check, ~5e-8
    /// error there). Used by buildRoots() to seed root_headings_ for the heading-constrained fine
    /// root prefilter (see prefilterRoots()).
    double yaw = 0.0;
  };

  /// @brief A branch-and-bound hypothesis paired with the roll/pitch offset that produced it.
  struct ScoredHypothesis
  {
    eidos::reloc::Hypothesis hyp;  ///< Scored 4-DOF pose hypothesis.
    double dr = 0.0;  ///< Roll offset (rad) applied to the query before search.
    double dp = 0.0;  ///< Pitch offset (rad) applied to the query before search.
  };

  /// @brief Best level-0 score (and its argmax pose) found for one root cell during the fine
  /// root-prefilter scan. Same order/index as `roots_` when produced by scoreRootsFine().
  struct FineRootScore
  {
    int score = -1;  ///< Best scorePoseAtLevel() value found for this root, or -1 if unscored
                      ///< (e.g. stop_requested_ fired mid-scan).
    std::size_t idx = 0;  ///< Index into `roots_` (and `root_headings_`).
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  ///< Argmax sub-position within the cell.
    double yaw = 0.0;  ///< Argmax yaw (rad) at that sub-position.
  };

  /**
   * @brief Resolve base_link <- lidar TF (once), buffer the de-tilted downsampled scan and query set.
   * @param msg Incoming PointCloud2 message.
   */
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Resolve base_link <- imu TF (once), extract roll/pitch for gravity alignment.
   * @param msg Incoming IMU message.
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// @brief Worker thread entry point: builds the pyramid (once), then searches and polishes.
  void workerMain();

  /**
   * @brief Rasterize prior-map keyframe clouds into the voxel pyramid and build corridor roots.
   *
   * Runs once. Populates `pyramid_`, `trajectory_`, `poses6d_`, and `roots_`. Never touches
   * MapManager::getKdTree() (non-const, unsafe off the SLAM thread) or getKeyFromCloudIndex()
   * (O(n) linear scan) -- uses getKeyList()/getCloudIndex() and a cached trajectory instead.
   *
   * @return True on success, false if no usable prior-map clouds were found (sets `pyramid_failed_`)
   *   or if `stop_requested_` fired mid-build.
   */
  bool buildPyramid();

  /**
   * @brief Try to rasterize one keyframe's points into `pyramid_` from a given data key.
   *
   * Prior-map clouds are stored under different concrete types depending on which pipeline stage
   * produced them (e.g. a raw `.../cloud` slot is PCL-typed, a downsampled `.../gicp_cloud` slot is
   * small_gicp-typed). This tries the PCL-typed retrieval first, then the small_gicp-typed
   * retrieval, so either source works regardless of the configured key's naming.
   *
   * When `use_free_space_` is set, this also casts a free-space ray from `sensor_offset_body`
   * (transformed into world frame by `world_t`, same as every point) to a strided subsample of the
   * keyframe's points, in the SAME pass as the occupancy insertion -- `VoxelPyramid::insertRay()`
   * is not thread-safe, and a second pass over the cloud would double the iteration cost for no
   * benefit.
   *
   * @param key GTSAM key identifying the keyframe.
   * @param data_key Data slot to retrieve from `map_manager_`.
   * @param world_t World-frame transform applied to each point before insertion.
   * @param sensor_offset_body Body-frame sensor origin used as the free-space ray origin (world
   *   origin is `world_t * sensor_offset_body`). See buildPyramid() for how this is derived.
   * @return True if a non-empty cloud was found under `data_key` (of either type) and inserted.
   */
  bool insertKeyframeCloud(
    gtsam::Key key, const std::string & data_key, const Eigen::Isometry3d & world_t,
    const Eigen::Vector3d & sensor_offset_body);

  /// @brief Build the coarsest-level corridor roots around `trajectory_`. Called once by buildPyramid().
  void buildRoots();

  /// @brief Publish a latched debug OccupancyGrid of the level-0 pyramid, if `publish_debug_grid_`.
  void publishDebugGrid();

  /**
   * @brief Search the roll/pitch offset grid and merge results into a globally NMS'd hypothesis list.
   * @param query Body-frame, gravity-de-tilted, downsampled live scan points.
   * @return Hypotheses sorted by score descending, mutually separated by `nms_radius_`.
   */
  std::vector<ScoredHypothesis> searchPoses(const std::vector<Eigen::Vector3d> & query);

  /**
   * @brief Build a query from a prior-map keyframe's own cloud, mirroring the live query path.
   *
   * Picks the prior-map keyframe whose position is closest to `near_position`, retrieves its
   * body-frame cloud, decomposes its recorded pose into roll/pitch/yaw, de-tilts the cloud by
   * `rotYX(pitch, roll)` -- the same de-tilt the live path applies to a fresh scan -- then applies
   * the identical `max_query_range_` horizontal filter, `inHeightBand()` height filter, and
   * strided downsample to `target_query_points_` that lidarCallback() applies to a live scan. The
   * result is, by construction, exactly registered to the map: it is literally a piece of the map,
   * scored at its own recorded pose.
   *
   * Used by the `debug_self_test_` block in searchPoses() (C1/C2/C3 scoring-only diagnostics) and
   * by `debug_use_self_query_` in workerMain() (substituting this query into the REAL search, to
   * isolate the search machinery from the scorer -- see workerMain() for why).
   *
   * @param near_position Map-frame point to search for the nearest prior-map keyframe around.
   * @param query_out Filled with the de-tilted, filtered, downsampled body-frame query points.
   * @param kf_translation_out Filled with the selected keyframe's map-frame translation.
   * @param kf_yaw_out Filled with the selected keyframe's recorded yaw (rad).
   * @param kf_index_out Filled with the selected keyframe's cloud index (for logging).
   * @return True on success; false if no prior-map keyframe or no usable cloud was found.
   */
  bool buildSelfQuery(
    const Eigen::Vector3d & near_position, std::vector<Eigen::Vector3d> & query_out,
    Eigen::Vector3d & kf_translation_out, double & kf_yaw_out, int & kf_index_out);

  /**
   * @brief Cheaply score every root and keep only the most promising ones before `branchAndBound()`.
   *
   * HEURISTIC, not exact: roots are ranked by a cheap score, so the true pose can in principle be
   * discarded if it ranks below `root_prefilter_keep_` other roots. A wrong survivor is still
   * rejected downstream by the GICP inlier gate and the uniqueness gate, so this is a ranking
   * shortcut, not a soundness-preserving filter.
   *
   * Two ranking modes, selected by `prefilter_fine_`:
   *  - false: the ORIGINAL coarse-level ranking (kept verbatim for A/B testing). Scores every root
   *    at the COARSEST pyramid level with a subsampled query. MEASURED to be non-discriminative:
   *    under DistanceField the per-point cell value ceiling is 255, so `root_prefilter_points_`
   *    points score at most `root_prefilter_points_ * 255`, and the coarsest level's dilation
   *    covers roughly 2.5x the cell size -- on the validation bag this saturated 11611 of 11613
   *    roots at the exact maximum score, so `std::nth_element`'s unspecified tie handling discarded
   *    the true root with about a 2% survival chance.
   *  - true (default): a FINE, heading-constrained level-0 ranking. Level 0 DOES discriminate
   *    (measured: the reference pose scored 94623 there vs. 77885 for the best wrong pose anywhere
   *    on the map, a 1.22x margin) -- see scoreRootsFine(). Also populates `prefilter_candidates_`
   *    with the argmax poses of the top-ranked roots, converted to real hypotheses that compete
   *    against branch-and-bound's own output in searchPoses()'s NMS step, since the coarse BnB
   *    bound provides no guidance once a root survives (it saturates at the same ceiling above).
   *
   * @param rotated_query Body-frame query already rotated by the current roll/pitch offset.
   * @param dr Roll offset (rad) applied to produce `rotated_query`; threaded into any
   *   `ScoredHypothesis` this call appends to `prefilter_candidates_`.
   * @param dp Pitch offset (rad) applied to produce `rotated_query`; see `dr`.
   * @return Up to `root_prefilter_keep_` roots, or all of `roots_` unchanged if
   *   `root_prefilter_keep_ <= 0` or is `>=` the number of roots (prefilter disabled).
   */
  std::vector<eidos::reloc::RootCell> prefilterRoots(
    const std::vector<Eigen::Vector3d> & rotated_query, double dr, double dp);

  /**
   * @brief Score every root cell's best (position, yaw) at pyramid level 0, within a heading-
   * constrained local search inside the cell.
   *
   * Shared VERBATIM by prefilterRoots()'s fine path and TRACE 9's diagnostic in searchPoses(), so
   * the ranking prefilterRoots() actually used and the ranking TRACE 9 reports on can never drift
   * apart. For each root, enumerates sub-positions spanning the root's coarsest-level cell (stepping
   * by `prefilter_xy_step_` in x/y and `prefilter_z_step_` in z, offset half a step inside the cell)
   * crossed with yaws (a +/-`heading_tolerance_deg_` window at `prefilter_yaw_step_deg_` around the
   * root's recorded heading when `use_heading_prior_`, plus the same window around heading+180deg
   * when `allow_reverse_heading_`, or the full circle at the same step when `use_heading_prior_` is
   * false), scores each with `scorePoseAtLevel()` at level 0 using `hit_weight_` (NOT the
   * hard-coded 3 the coarse path uses -- see prefilterRoots()'s doc), and keeps the best score and
   * its argmax pose.
   *
   * @param sub_query Subsampled, rotated query points to score against (same strided subsample
   *   `prefilterRoots()` builds from `root_prefilter_points_`).
   * @return One FineRootScore per root in `roots_`, same order/index as `roots_`.
   */
  std::vector<FineRootScore> scoreRootsFine(const std::vector<Eigen::Vector3d> & sub_query) const;

  /**
   * @brief Refine the top hypotheses with GICP and apply the acceptance gates.
   * @param hypotheses Search results from searchPoses(), sorted by score descending.
   * @return RelocalizationResult for the first hypothesis clearing all gates, std::nullopt otherwise.
   */
  std::optional<RelocalizationResult> gicpPolish(const std::vector<ScoredHypothesis> & hypotheses);

  /**
   * @brief Release `pyramid_`'s memory and log the RSS change around the release.
   *
   * MEMORY RECOVERY is a hard requirement, not a nicety: the user accepted the distance-field
   * pyramid's larger memory cost only on the condition that it is actually recoverable once the
   * search is done with it. Freeing the pyramid's buffers (`VoxelPyramid::releaseMemory()`, which
   * additionally `malloc_trim()`s under glibc) does not by itself guarantee the OS reclaims that
   * memory back from the process -- freed heap normally stays in the allocator's arena rather than
   * being returned -- so this logs process RSS immediately before and after the release, which is
   * what makes "the memory actually came back" checkable in a field log instead of merely assumed
   * because releaseMemory() was called. A no-op (and no log) when the pyramid is already empty, so
   * repeated calls along different exit paths (lock, deactivate, destructor) log at most once.
   *
   * @param context Short label for which exit path triggered the release (e.g. "lock",
   *   "deactivate", "destructor"), included in the log line.
   */
  void releasePyramidMemory(const char * context);

  // Subscriptions / publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_pub_;

  // Buffered sensor data -- written by callbacks, read by the worker thread.
  small_gicp::PointCloud::Ptr latest_scan_;  ///< Downsampled body-frame scan, for GICP polish.
  std::vector<Eigen::Vector3d> latest_query_;  ///< De-tilted, subsampled scan, for BnB search.
  std::mutex scan_lock_;

  double latest_imu_roll_ = 0.0;
  double latest_imu_pitch_ = 0.0;
  std::mutex imu_lock_;
  bool has_imu_ = false;

  std::atomic<bool> active_{false};

  // base_link <- lidar TF (resolved once)
  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_lidar_tf_ = false;

  // base_link <- imu TF (resolved once, rotation only)
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;

  std::string base_link_frame_;
  std::string map_frame_;
  std::string lidar_frame_;
  std::string imu_frame_;

  // ---- Threading: launch-or-poll protocol ----
  std::atomic<bool> search_running_{false};
  std::atomic<bool> result_ready_{false};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::optional<RelocalizationResult> result_;
  std::mutex result_mtx_;

  // ---- Worker-owned state (touched only by the worker thread) ----
  eidos::reloc::VoxelPyramid pyramid_;
  bool pyramid_built_ = false;
  bool pyramid_failed_ = false;
  pcl::PointCloud<PoseType>::Ptr poses6d_;
  std::vector<TrajectoryEntry> trajectory_;
  std::vector<eidos::reloc::RootCell> roots_;
  /// Map-frame heading (yaw, rad, stored as float) of the trajectory entry that FIRST admitted
  /// each cell in `roots_` -- same length and index as `roots_`, populated by buildRoots(). Feeds
  /// the fine prefilter's heading-constrained yaw window (see prefilterRoots()/scoreRootsFine()).
  /// Cleared everywhere `roots_` is cleared.
  std::vector<float> root_headings_;
  /// Argmax poses from the fine root prefilter's top-ranked roots, converted to real hypotheses
  /// (see prefilterRoots()). Cleared at the start of each searchPoses() call and repopulated once
  /// per roll/pitch offset; merged into the candidate list in searchPoses() BEFORE NMS so they
  /// compete against branch-and-bound's own output on equal footing.
  std::vector<ScoredHypothesis> prefilter_candidates_;

  /// @brief Mean normalized score of randomly drawn corridor poses for the most recent search.
  ///
  /// The "no real match" level of the active scoring function, measured rather than assumed:
  /// neither scoring mode bottoms out at zero (distance_field floors near 0.56 on a dense map,
  /// occupancy at 1/hit_weight), so the uniqueness gate in gicpPolish() compares scores in excess
  /// of this floor instead of as a raw quotient. 0.0 when it could not be estimated, which
  /// degrades the gate to the original raw-quotient behaviour.
  double last_chance_floor_ = 0.0;
  /// Whether the free-space raycast actually ran for this build. Under score_mode=distance_field
  /// the free-space channel is never built (the distance field subsumes it -- see score_mode_
  /// below), so this can be false even when use_free_space_ is true; insertKeyframeCloud() checks
  /// this rather than use_free_space_ directly so the raycast work is skipped entirely, not just
  /// made a pyramid-side no-op.
  bool build_free_space_this_run_ = false;
  /// The score mode the pyramid was ACTUALLY built with, which can differ from score_mode_ if
  /// max_score_voxels_ was exceeded (see buildPyramid()). Everything downstream of the build --
  /// SearchConfig::score_mode and every probe diagnostic -- reads this, not score_mode_, so a
  /// silent fallback can never be misreported as the requested mode.
  eidos::reloc::ScoreMode active_score_mode_ = eidos::reloc::ScoreMode::DistanceField;

  // ---- Parameters ----
  std::string pointcloud_from_;
  bool prefer_downsampled_source_ = false;
  double min_voxel_size_ = 1.0;
  int num_levels_ = 4;
  // ---- Distance-field scoring ----
  // Measured on ring_road.map: binary occupancy carries almost no alignment signal there (a
  // randomly rotated scan already hits 76% of query points at +/-1 voxel, and the true yaw ranks
  // only 5th of 36 -- see the doc page's Status section). A distance field keeps the graded
  // distinction between "on structure" and "near structure" that binary containment throws away,
  // which is also what lets GICP succeed on this same data where the voxel test does not.
  /// "distance_field" (new default) or "occupancy" (today's ternary score, kept for A/B testing
  /// the two scorers against the same map and scan). Parsed by parseScoreMode() in the .cpp; an
  /// unrecognised string WARNs and falls back to "distance_field" rather than throwing.
  eidos::reloc::ScoreMode score_mode_ = eidos::reloc::ScoreMode::DistanceField;
  double df_sigma_ = 1.0;  ///< Distance-field falloff (m); ~= the measured registration offset to absorb.
  int df_truncation_voxels_ = 2;  ///< Distance-field kernel radius, in level-0 voxels.
  std::size_t max_score_voxels_ = 40000000;  ///< Guard on the distance-field grid; on overflow,
                                              ///< buildPyramid() falls back to occupancy and WARNs.
  double min_height_ = 0.6;  ///< Body-frame z lower bound (m); removes ground returns.
  double max_height_ = 6.0;
  std::size_t max_voxels_per_level_ = 8000000;
  int target_query_points_ = 400;
  double max_query_range_ = 40.0;
  std::vector<double> debug_probe_pose_;  ///< Optional [x,y,z,yaw_deg] diagnostic probe.  ///< Drop query points beyond this horizontal range (m); <=0 disables.
  /// TEMPORARY diagnostic switch: when true (and `debug_probe_pose_` is set), runs the
  /// height-band discrimination sweep in searchPoses() -- z histograms plus a per-band yaw/xy
  /// discrimination sweep -- to determine whether the configured body-frame height band is
  /// discarding ground-level structure and keeping mostly tree canopy. See the block's doc
  /// comment in the .cpp for details. Defaults to false: zero cost, zero behaviour change.
  bool debug_band_sweep_ = false;
  /// TEMPORARY diagnostic switch: when true (and `debug_probe_pose_` is set), runs the
  /// resolution/scoring-mode/min-observation-count sweep in searchPoses() -- immediately after
  /// the debug_band_sweep_ block -- to determine whether the prior map is simply space-filling
  /// at the current 1.0 m level-0 resolution (the band sweep exonerated the height band: even
  /// the best band only reached yaw_rank 10.8/36 vs. a random expectation of 18.5). Unlike the
  /// band sweep, this one also reports WHERE each case's score peaks (yaw/xy/z argmax), not just
  /// the reference pose's rank, because a rank far from 1 is ambiguous on its own -- it could
  /// mean "no peak anywhere" or "a real peak sitting at a displaced pose" (GICP from the
  /// reference moved 0.58 m, so a nonzero-but-consistent argmax would be a real offset, not
  /// noise). See the block's doc comment in the .cpp for the full case list and rationale.
  /// Defaults to false: zero cost, zero behaviour change.
  bool debug_res_sweep_ = false;
  /// TEMPORARY diagnostic switch: when true (and `debug_probe_pose_` is set), runs the
  /// self-test control in searchPoses() -- immediately after the debug_res_sweep_ block --
  /// which scores a prior-map keyframe's OWN cloud at its OWN pose. Every measurement made
  /// with debug_band_sweep_/debug_res_sweep_ ran the LIVE scan through TF extrinsics, IMU
  /// de-tilt, downsampling and the height band before scoring, so none of them can
  /// distinguish "the scene has no x/y/yaw signal" from "the live-query path is corrupting
  /// it". This control removes every one of those suspects at once: the query is drawn
  /// straight from the map itself, so it is exactly registered by construction. C1 scores
  /// it against the pyramid that INCLUDES the keyframe (a plumbing sanity check -- this
  /// must land at ~100% exact hit and yaw_rank 1/36, or there is a transform/scoring bug);
  /// C2/C3 score it against a local map that EXCLUDES the keyframe and its neighbours (the
  /// honest single-scan test of whether this scene has any x/y/yaw signal at all). See the
  /// block's doc comment in the .cpp for the full procedure. Defaults to false: zero cost,
  /// zero behaviour change.
  bool debug_self_test_ = false;
  /// TEMPORARY diagnostic switch: when true (and `debug_probe_pose_` is set), workerMain()
  /// substitutes a keyframe self-query (built by buildSelfQuery(), seeded at `debug_probe_pose_`'s
  /// translation) for the live scan BEFORE the real search runs. Unlike debug_self_test_, which
  /// only scores the self-query in isolation (searchPoses()'s own scoring sweeps), this routes it
  /// through the FULL production path -- corridor roots, prefilter, branch-and-bound, NMS,
  /// GICP -- exactly like a live search, just with a query that is known to score perfectly at
  /// its reference pose. That isolates one remaining question: given a query with a real,
  /// verified peak, can the search MACHINERY itself find it end-to-end (as opposed to the scorer,
  /// which debug_self_test_ already verified separately)? Defaults to false: zero cost, zero
  /// behaviour change.
  bool debug_use_self_query_ = false;
  double rp_search_range_ = 0.02;
  int rp_search_steps_ = 1;  // 1 = trust IMU gravity; each extra step multiplies search cost
  double search_corridor_ = 30.0;
  double z_margin_ = 5.0;
  double prune_slack_ = 0.8;
  int max_search_nodes_ = 200000;  ///< Total BnB node budget per search, split across tasks.
  double nms_radius_ = 5.0;
  int root_prefilter_points_ = 128;
  int root_prefilter_keep_ = 256;
  // ---- Fine, heading-constrained root prefilter ----
  // MEASURED root cause of a 281 m mislocalization on the validation bag: the coarse-level
  // prefilter (root_prefilter_points_ points scored at the COARSEST pyramid level) saturates at
  // its ceiling (root_prefilter_points_ * 255 under DistanceField) for nearly every root near the
  // road -- 11611 of 11613 roots tied at the exact maximum -- so std::nth_element's unspecified
  // handling of ties discarded the true root with about a 2% survival chance. Level 0 DOES
  // discriminate (measured 94623 at the reference pose vs. 77885 for the best wrong pose anywhere
  // on the map), so the fine prefilter ranks roots by a heading-constrained level-0 scan instead.
  // See prefilterRoots()'s doc comment for the full argument.
  bool prefilter_fine_ = true;  ///< false = exactly the old coarse prefilter (kept for A/B testing).
  bool use_heading_prior_ = true;  ///< Constrain the fine scan's yaw window to each root's recorded heading.
  double heading_tolerance_deg_ = 30.0;  ///< +/- yaw window (deg) around a root's recorded heading.
  bool allow_reverse_heading_ = true;  ///< Also scan the +/-tolerance window around heading+180deg
                                        ///< (the vehicle may have traversed that keyframe's cell
                                        ///< heading the opposite way on a different pass).
  double prefilter_xy_step_ = 2.0;  ///< Sub-position spacing inside a root cell, x/y (m).
  double prefilter_z_step_ = 4.0;  ///< Sub-position spacing inside a root cell, z (m).
  double prefilter_yaw_step_deg_ = 10.0;  ///< Yaw step (deg) within the heading window (or full circle).
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

  // ---- Free-space channel parameters ----
  // A query point landing in known-empty space is negative evidence (it should have hit the map
  // if the pose were right), not merely absent evidence -- see the ternary-score contract in
  // bnb_search.hpp for why this exists and the soundness argument for the pyramid-side bound.
  /// Master switch for the known-free-space channel; false reproduces the plain binary-hit score
  /// exactly. Defaults OFF: measured on ring_road.map it did not help, because the underlying
  /// occupancy score was found to be non-discriminative there in the first place (a randomly
  /// rotated scan already hits 76% of the time at +/-1 voxel, so the band is nearly space-filling).
  /// That makes the free-space comparison on that map inconclusive rather than negative -- the
  /// channel is sound and stays available, but it is not enabled by default on unproven evidence.
  bool use_free_space_ = false;
  int free_rays_per_keyframe_ = 2000;  ///< Stride each keyframe's cloud to about this many rays; <=0 uses every point.
  double free_max_range_ = 40.0;  ///< Clamp free-space ray length (m); bounds raycast cost.
  double free_end_margin_ = 1.0;  ///< Stop each ray this many metres short of its endpoint (m).
  bool free_clear_near_occupied_ = true;  ///< Delete free voxels occupied or 26-adjacent to occupied.
  std::size_t max_free_voxels_ = 20000000;  ///< Budget guard; over this, free space is abandoned entirely.
  double free_ray_origin_height_ = 2.0;  ///< Fallback ray-origin height (m) above base_link when the
                                          ///< base_link<-lidar TF has not resolved yet at build time.
  int hit_weight_ = 3;  ///< Per-point weight W for an occupied hit in the ternary score (raw in [0, W*n]).
};

}  // namespace eidos
