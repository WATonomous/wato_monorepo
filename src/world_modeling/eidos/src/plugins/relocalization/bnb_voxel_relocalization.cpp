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

#include "eidos/plugins/relocalization/bnb_voxel_relocalization.hpp"

#include <tf2_ros/buffer.h>

#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/map/map_manager.hpp"
#include "eidos/utils/conversions.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos
{

namespace
{

/**
 * @brief Compose Ry(pitch) * Rx(roll), the same de-tilt/re-tilt convention used throughout this
 * plugin (mirrors the gravity-alignment math in GpsIcpRelocalization::imuCallback()).
 * @param pitch Pitch angle (rad).
 * @param roll Roll angle (rad).
 * @return Rotation matrix Ry(pitch) * Rx(roll).
 */
Eigen::Matrix3d rotYX(double pitch, double roll)
{
  Eigen::Matrix3d rx;
  rx << 1, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, std::sin(roll), std::cos(roll);
  Eigen::Matrix3d ry;
  ry << std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(pitch), 0, std::cos(pitch);
  return ry * rx;
}

/**
 * @brief Split a root list into up to `num_chunks` contiguous, near-equal-size chunks.
 *
 * Used to spread branch-and-bound's parallel work across roots rather than only across
 * roll/pitch offsets. Returns fewer than `num_chunks` chunks (never empty ones) when there are
 * fewer roots than chunks requested.
 *
 * @param roots Roots to split.
 * @param num_chunks Requested chunk count (typically `num_threads_`).
 * @return Non-empty chunks, in order, covering every root in `roots` exactly once.
 */
std::vector<std::vector<eidos::reloc::RootCell>> chunkRoots(
  const std::vector<eidos::reloc::RootCell> & roots, int num_chunks)
{
  std::vector<std::vector<eidos::reloc::RootCell>> chunks;
  if (roots.empty() || num_chunks <= 0) return chunks;

  const std::size_t n = roots.size();
  const std::size_t chunk_count = std::min(static_cast<std::size_t>(num_chunks), n);
  chunks.resize(chunk_count);
  const std::size_t base = n / chunk_count;
  const std::size_t rem = n % chunk_count;

  std::size_t idx = 0;
  for (std::size_t c = 0; c < chunk_count; ++c) {
    const std::size_t sz = base + (c < rem ? 1 : 0);
    chunks[c].assign(
      roots.begin() + static_cast<std::ptrdiff_t>(idx), roots.begin() + static_cast<std::ptrdiff_t>(idx + sz));
    idx += sz;
  }
  return chunks;
}

/**
 * @brief Stride for subsampling a keyframe's points down to about `target` free-space rays.
 * @param n Number of points in the keyframe cloud.
 * @param target Desired ray count; `<= 0` means "use every point" (stride 1).
 * @return Stride to iterate the cloud with (always >= 1).
 */
std::size_t raycastStride(std::size_t n, int target)
{
  if (target <= 0 || n == 0) return 1;
  const std::size_t t = static_cast<std::size_t>(target);
  return std::max<std::size_t>(1, n / t);
}

/**
 * @brief Parse the `score_mode` parameter string into `eidos::reloc::ScoreMode`.
 *
 * An unrecognised value WARNs and falls back to DistanceField (the new default) rather than
 * throwing, so a typo in a launch file degrades gracefully instead of crashing the node.
 *
 * @param raw Raw parameter string: "distance_field" or "occupancy".
 * @param logger Logger for the fallback warning.
 * @param name Plugin instance name, for the log prefix.
 * @return Parsed score mode.
 */
eidos::reloc::ScoreMode parseScoreMode(
  const std::string & raw, const rclcpp::Logger & logger, const std::string & name)
{
  if (raw == "distance_field") return eidos::reloc::ScoreMode::DistanceField;
  if (raw == "occupancy") return eidos::reloc::ScoreMode::Occupancy;
  RCLCPP_WARN(
    logger,
    "[%s] unrecognised score_mode '%s' (expected 'distance_field' or 'occupancy'), falling back to "
    "'distance_field'",
    name.c_str(),
    raw.c_str());
  return eidos::reloc::ScoreMode::DistanceField;
}

/// @brief Human-readable label for a score mode, for log lines.
/// @param mode Score mode to label.
/// @return "distance_field" or "occupancy".
const char * scoreModeLabel(eidos::reloc::ScoreMode mode)
{
  return mode == eidos::reloc::ScoreMode::DistanceField ? "distance_field" : "occupancy";
}

/**
 * @brief Read the process's current resident set size from /proc/self/status.
 *
 * Used only around releasePyramidMemory()'s calls to VoxelPyramid::releaseMemory(): freeing the
 * pyramid's buffers does not by itself guarantee the OS reclaims that memory (freed heap normally
 * stays in the allocator's arena rather than being returned), so logging RSS immediately before and
 * after is what makes "the memory actually came back" checkable in a field log instead of merely
 * assumed because release was called.
 *
 * @return RSS in megabytes, or 0.0 if /proc/self/status could not be read (non-Linux, sandboxed).
 */
double currentRssMb()
{
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream iss(line.substr(6));
      double kb = 0.0;
      iss >> kb;
      return kb / 1024.0;
    }
  }
  return 0.0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void BnbVoxelRelocalization::onInitialize()
{
  std::string prefix = name_;

  node_->declare_parameter(prefix + ".lidar_topic", std::string("/lidar/points"));
  node_->declare_parameter(prefix + ".imu_topic", std::string("/imu/data"));
  node_->declare_parameter(prefix + ".lidar_frame", std::string("lidar"));
  node_->declare_parameter(prefix + ".imu_frame", std::string("imu_link"));
  node_->declare_parameter(prefix + ".pointcloud_from", std::string("liso_factor/cloud"));
  node_->declare_parameter(prefix + ".prefer_downsampled_source", prefer_downsampled_source_);
  node_->declare_parameter(prefix + ".min_voxel_size", min_voxel_size_);
  // Default to 4, not 6: measurement showed the branch-and-bound bound saturates and carries no
  // information at coarse resolutions -- with a 40 m-range query, levels at 16 m/32 m/64 m scored
  // ~100% of query points for essentially every candidate pose, because 26-neighbourhood dilation
  // of a 32 m voxel covers ~96 m. Levels coarser than ~8 m only multiply work without pruning.
  node_->declare_parameter(prefix + ".num_levels", num_levels_);
  node_->declare_parameter(prefix + ".score_mode", std::string("distance_field"));
  node_->declare_parameter(prefix + ".df_sigma", df_sigma_);
  node_->declare_parameter(prefix + ".df_truncation_voxels", df_truncation_voxels_);
  node_->declare_parameter(prefix + ".max_score_voxels", static_cast<int>(max_score_voxels_));
  node_->declare_parameter(prefix + ".max_height", max_height_);
  node_->declare_parameter(prefix + ".min_height", min_height_);
  node_->declare_parameter(prefix + ".max_voxels_per_level", static_cast<int>(max_voxels_per_level_));
  node_->declare_parameter(prefix + ".target_query_points", target_query_points_);
  node_->declare_parameter(prefix + ".max_query_range", max_query_range_);
  node_->declare_parameter(prefix + ".debug_probe_pose", std::vector<double>{});
  node_->declare_parameter(prefix + ".debug_band_sweep", debug_band_sweep_);
  node_->declare_parameter(prefix + ".debug_res_sweep", debug_res_sweep_);
  node_->declare_parameter(prefix + ".debug_self_test", debug_self_test_);
  node_->declare_parameter(prefix + ".debug_use_self_query", debug_use_self_query_);
  node_->declare_parameter(prefix + ".rp_search_range", rp_search_range_);
  node_->declare_parameter(prefix + ".rp_search_steps", rp_search_steps_);
  node_->declare_parameter(prefix + ".search_corridor", search_corridor_);
  node_->declare_parameter(prefix + ".z_margin", z_margin_);
  node_->declare_parameter(prefix + ".prune_slack", prune_slack_);
  node_->declare_parameter(prefix + ".max_search_nodes", max_search_nodes_);
  node_->declare_parameter(prefix + ".nms_radius", nms_radius_);
  node_->declare_parameter(prefix + ".root_prefilter_points", root_prefilter_points_);
  node_->declare_parameter(prefix + ".root_prefilter_keep", root_prefilter_keep_);
  node_->declare_parameter(prefix + ".prefilter_fine", prefilter_fine_);
  node_->declare_parameter(prefix + ".use_heading_prior", use_heading_prior_);
  node_->declare_parameter(prefix + ".heading_tolerance_deg", heading_tolerance_deg_);
  node_->declare_parameter(prefix + ".allow_reverse_heading", allow_reverse_heading_);
  node_->declare_parameter(prefix + ".prefilter_xy_step", prefilter_xy_step_);
  node_->declare_parameter(prefix + ".prefilter_z_step", prefilter_z_step_);
  node_->declare_parameter(prefix + ".prefilter_yaw_step_deg", prefilter_yaw_step_deg_);
  node_->declare_parameter(prefix + ".min_match_score", min_match_score_);
  node_->declare_parameter(prefix + ".min_score_ratio", min_score_ratio_);
  node_->declare_parameter(prefix + ".num_gicp_candidates", num_gicp_candidates_);
  node_->declare_parameter(prefix + ".min_inlier_ratio", min_inlier_ratio_);
  node_->declare_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->declare_parameter(prefix + ".submap_radius", submap_radius_);
  node_->declare_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_icp_iterations", max_icp_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".publish_debug_grid", publish_debug_grid_);
  node_->declare_parameter(prefix + ".debug_grid_topic", std::string("slam/visualization/reloc_voxel_grid"));
  node_->declare_parameter(prefix + ".use_free_space", use_free_space_);
  node_->declare_parameter(prefix + ".free_rays_per_keyframe", free_rays_per_keyframe_);
  node_->declare_parameter(prefix + ".free_max_range", free_max_range_);
  node_->declare_parameter(prefix + ".free_end_margin", free_end_margin_);
  node_->declare_parameter(prefix + ".free_clear_near_occupied", free_clear_near_occupied_);
  node_->declare_parameter(prefix + ".max_free_voxels", static_cast<int>(max_free_voxels_));
  node_->declare_parameter(prefix + ".free_ray_origin_height", free_ray_origin_height_);
  node_->declare_parameter(prefix + ".hit_weight", hit_weight_);

  std::string lidar_topic, imu_topic;
  int max_voxels_per_level_int = static_cast<int>(max_voxels_per_level_);
  node_->get_parameter(prefix + ".lidar_topic", lidar_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".prefer_downsampled_source", prefer_downsampled_source_);
  node_->get_parameter(prefix + ".min_voxel_size", min_voxel_size_);
  node_->get_parameter(prefix + ".num_levels", num_levels_);
  std::string score_mode_str;
  node_->get_parameter(prefix + ".score_mode", score_mode_str);
  score_mode_ = parseScoreMode(score_mode_str, node_->get_logger(), name_);
  active_score_mode_ = score_mode_;  // provisional; buildPyramid() may revise this on overflow.
  node_->get_parameter(prefix + ".df_sigma", df_sigma_);
  node_->get_parameter(prefix + ".df_truncation_voxels", df_truncation_voxels_);
  int max_score_voxels_int = static_cast<int>(max_score_voxels_);
  node_->get_parameter(prefix + ".max_score_voxels", max_score_voxels_int);
  max_score_voxels_ = static_cast<std::size_t>(max_score_voxels_int);
  node_->get_parameter(prefix + ".max_height", max_height_);
  node_->get_parameter(prefix + ".min_height", min_height_);
  node_->get_parameter(prefix + ".max_voxels_per_level", max_voxels_per_level_int);
  max_voxels_per_level_ = static_cast<std::size_t>(max_voxels_per_level_int);
  node_->get_parameter(prefix + ".target_query_points", target_query_points_);
  node_->get_parameter(prefix + ".max_query_range", max_query_range_);
  node_->get_parameter(prefix + ".debug_probe_pose", debug_probe_pose_);
  node_->get_parameter(prefix + ".debug_band_sweep", debug_band_sweep_);
  node_->get_parameter(prefix + ".debug_res_sweep", debug_res_sweep_);
  node_->get_parameter(prefix + ".debug_self_test", debug_self_test_);
  node_->get_parameter(prefix + ".debug_use_self_query", debug_use_self_query_);
  node_->get_parameter(prefix + ".rp_search_range", rp_search_range_);
  node_->get_parameter(prefix + ".rp_search_steps", rp_search_steps_);
  node_->get_parameter(prefix + ".search_corridor", search_corridor_);
  node_->get_parameter(prefix + ".z_margin", z_margin_);
  node_->get_parameter(prefix + ".prune_slack", prune_slack_);
  node_->get_parameter(prefix + ".max_search_nodes", max_search_nodes_);
  node_->get_parameter(prefix + ".nms_radius", nms_radius_);
  node_->get_parameter(prefix + ".root_prefilter_points", root_prefilter_points_);
  node_->get_parameter(prefix + ".root_prefilter_keep", root_prefilter_keep_);
  node_->get_parameter(prefix + ".prefilter_fine", prefilter_fine_);
  node_->get_parameter(prefix + ".use_heading_prior", use_heading_prior_);
  node_->get_parameter(prefix + ".heading_tolerance_deg", heading_tolerance_deg_);
  node_->get_parameter(prefix + ".allow_reverse_heading", allow_reverse_heading_);
  node_->get_parameter(prefix + ".prefilter_xy_step", prefilter_xy_step_);
  node_->get_parameter(prefix + ".prefilter_z_step", prefilter_z_step_);
  node_->get_parameter(prefix + ".prefilter_yaw_step_deg", prefilter_yaw_step_deg_);
  node_->get_parameter(prefix + ".min_match_score", min_match_score_);
  node_->get_parameter(prefix + ".min_score_ratio", min_score_ratio_);
  node_->get_parameter(prefix + ".num_gicp_candidates", num_gicp_candidates_);
  node_->get_parameter(prefix + ".min_inlier_ratio", min_inlier_ratio_);
  node_->get_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->get_parameter(prefix + ".submap_radius", submap_radius_);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_icp_iterations", max_icp_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".publish_debug_grid", publish_debug_grid_);
  node_->get_parameter(prefix + ".debug_grid_topic", debug_grid_topic_);
  node_->get_parameter(prefix + ".use_free_space", use_free_space_);
  node_->get_parameter(prefix + ".free_rays_per_keyframe", free_rays_per_keyframe_);
  node_->get_parameter(prefix + ".free_max_range", free_max_range_);
  node_->get_parameter(prefix + ".free_end_margin", free_end_margin_);
  node_->get_parameter(prefix + ".free_clear_near_occupied", free_clear_near_occupied_);
  int max_free_voxels_int = static_cast<int>(max_free_voxels_);
  node_->get_parameter(prefix + ".max_free_voxels", max_free_voxels_int);
  max_free_voxels_ = static_cast<std::size_t>(max_free_voxels_int);
  node_->get_parameter(prefix + ".free_ray_origin_height", free_ray_origin_height_);
  node_->get_parameter(prefix + ".hit_weight", hit_weight_);
  node_->get_parameter("frames.base_link", base_link_frame_);
  node_->get_parameter("frames.map", map_frame_);

  if (use_free_space_ && score_mode_ == eidos::reloc::ScoreMode::DistanceField) {
    // A distance field already scores a point far from all structure at ~0, continuously, which
    // is exactly what the three-state occupancy score was approximating with its known-free
    // channel -- so under distance_field the channel is redundant, not merely unused, and is
    // skipped entirely at build time (see buildPyramid()) rather than built and ignored.
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] use_free_space is set but score_mode=distance_field: the free-space channel is inactive "
      "(the distance field subsumes it) -- it will not be built or raycast this session",
      name_.c_str());
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&BnbVoxelRelocalization::lidarCallback, this, std::placeholders::_1),
    sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&BnbVoxelRelocalization::imuCallback, this, std::placeholders::_1),
    sub_opts);

  if (publish_debug_grid_) {
    debug_grid_pub_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>(debug_grid_topic_, rclcpp::QoS(1).transient_local());
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] initialized (BnB voxel, lidar=%s, num_levels=%d, score_mode=%s)",
    name_.c_str(),
    lidar_topic.c_str(),
    num_levels_,
    scoreModeLabel(score_mode_));
}

void BnbVoxelRelocalization::activate()
{
  active_ = true;
  stop_requested_ = false;
  if (debug_grid_pub_) debug_grid_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void BnbVoxelRelocalization::deactivate()
{
  active_ = false;
  stop_requested_ = true;
  if (worker_.joinable()) worker_.join();
  // Deactivation ends this session's use of the pyramid -- release it (see releasePyramidMemory())
  // and reset the build flags so a future activate() rebuilds fresh instead of searching an
  // emptied pyramid. Safe without additional synchronisation: the worker was just joined above, so
  // nothing else touches this worker-owned state concurrently.
  releasePyramidMemory("deactivate");
  roots_.clear();
  roots_.shrink_to_fit();
  root_headings_.clear();
  root_headings_.shrink_to_fit();
  prefilter_candidates_.clear();
  prefilter_candidates_.shrink_to_fit();
  pyramid_built_ = false;
  pyramid_failed_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

BnbVoxelRelocalization::~BnbVoxelRelocalization()
{
  stop_requested_ = true;
  if (worker_.joinable()) worker_.join();
  releasePyramidMemory("destructor");
  roots_.clear();
  roots_.shrink_to_fit();
  root_headings_.clear();
  root_headings_.shrink_to_fit();
  prefilter_candidates_.clear();
  prefilter_candidates_.shrink_to_fit();
}

void BnbVoxelRelocalization::releasePyramidMemory(const char * context)
{
  if (pyramid_.empty()) return;  // Nothing to release; avoid a spurious RSS log on every exit path.
  const double rss_before_mb = currentRssMb();
  pyramid_.releaseMemory();
  const double rss_after_mb = currentRssMb();
  // See currentRssMb()'s doc comment for why this is measured rather than assumed: freeing the
  // pyramid's buffers does not by itself return that memory to the OS, which is exactly the
  // failure mode this log is required to guard against.
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] pyramid released (%s): RSS %.0f MB -> %.0f MB",
    name_.c_str(),
    context,
    rss_before_mb,
    rss_after_mb);
}

// ---------------------------------------------------------------------------
// Callbacks — buffer latest sensor data
// ---------------------------------------------------------------------------
void BnbVoxelRelocalization::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Resolve base_link ← lidar TF (once)
  if (!has_lidar_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, lidar_frame_, tf2::TimePointZero);
      const auto & t = tf_msg.transform.translation;
      const auto & r = tf_msg.transform.rotation;
      T_base_lidar_ = Eigen::Isometry3d::Identity();
      T_base_lidar_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      T_base_lidar_.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_lidar_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  // Convert to small_gicp and transform to body frame
  auto raw = fromRosMsg(*msg);
  if (!raw || raw->empty()) return;

  for (size_t i = 0; i < raw->size(); i++) {
    Eigen::Vector4d & pt = raw->point(i);
    pt.head<3>() = T_base_lidar_ * pt.head<3>();
  }

  auto [ds, tree] = small_gicp::preprocess_points(*raw, scan_ds_resolution_, num_neighbors_, num_threads_);

  // Build the de-tilted BnB query set from the same downsampled cloud used for GICP, since the
  // search cost is linear in query size and full-resolution points would dominate runtime.
  double roll = 0.0, pitch = 0.0;
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    roll = latest_imu_roll_;
    pitch = latest_imu_pitch_;
  }
  // De-tilt takes a point from the tilted body frame into a gravity-levelled frame, which is
  // R_level_body = Ry(pitch) * Rx(roll) -- the POSITIVE angles. (Negating them tilts the wrong
  // way and doubles the error instead of removing it: at 40 m a 2 deg error displaces a point
  // 1.4 m, more than one voxel at the default resolution, so far returns miss systematically.)
  // Combined with the search's Rz(yaw) this reproduces gtsam's RzRyRx(roll, pitch, yaw).
  const Eigen::Matrix3d r_detilt = rotYX(pitch, roll);

  // Points beyond `max_query_range` are dropped. This matters for more than cost: the coarsest
  // yaw-bin count is 2*pi / (r_coarse / max_range), so a 100 m sensor range inflates the seed set
  // (measured: 79 bins per root at 8 m coarsest) until the node budget is spent before the search
  // can descend to any leaf. Long-range returns are also the least reliable to match, since the
  // prior map is rasterized from voxel-downsampled clouds.
  const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
  std::vector<Eigen::Vector3d> query;
  query.reserve(ds->size());
  for (size_t i = 0; i < ds->size(); i++) {
    const Eigen::Vector3d p = r_detilt * ds->point(i).head<3>();
    if (max_range_sq > 0.0 && p.head<2>().squaredNorm() > max_range_sq) continue;
    // Same body-frame band as the map rasterization. Ground returns are the least
    // discriminative part of a scan -- they match almost anywhere -- so keeping them makes the
    // occupancy score reflect map coverage density rather than alignment quality.
    if (!inHeightBand(p.z())) continue;
    query.push_back(p);
  }

  if (target_query_points_ > 0 && static_cast<int>(query.size()) > target_query_points_) {
    const std::size_t target = static_cast<std::size_t>(target_query_points_);
    const std::size_t n = query.size();
    const double step = static_cast<double>(n) / static_cast<double>(target);
    std::vector<Eigen::Vector3d> strided;
    strided.reserve(target);
    for (std::size_t k = 0; k < target; ++k) {
      std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
      if (idx >= n) idx = n - 1;
      strided.push_back(query[idx]);
    }
    query = std::move(strided);
  }

  std::lock_guard<std::mutex> lock(scan_lock_);
  latest_scan_ = ds;
  latest_query_ = std::move(query);
}

void BnbVoxelRelocalization::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Resolve base_link ← imu TF (once, rotation only)
  if (!has_imu_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, imu_frame_, tf2::TimePointZero);
      const auto & r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_imu_tf_ = true;
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  // IMU quaternion gives R_world_imu. Transform to body frame:
  // R_world_base = R_world_imu * R_imu_base = R_world_imu * inv(R_base_imu)
  Eigen::Quaterniond q_imu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  if (q_imu.squaredNorm() < 0.5) return;
  q_imu.normalize();

  Eigen::Matrix3d R_world_base = q_imu.toRotationMatrix() * R_base_imu_.transpose();

  // Extract roll/pitch from body-frame rotation (yaw is unobserved by gravity alone).
  gtsam::Rot3 body_rot(R_world_base);

  std::lock_guard<std::mutex> lock(imu_lock_);
  latest_imu_roll_ = body_rot.roll();
  latest_imu_pitch_ = body_rot.pitch();
  has_imu_ = true;
}

// ---------------------------------------------------------------------------
// tryRelocalize — launch-or-poll only, never blocks
// ---------------------------------------------------------------------------
std::optional<RelocalizationResult> BnbVoxelRelocalization::tryRelocalize(  // NOLINT(readability/casting)
  double /*timestamp*/)
{
  if (!active_) return std::nullopt;

  if (!map_manager_->hasPriorMap()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[%s] no prior map loaded", name_.c_str());
    return std::nullopt;
  }

  if (result_ready_.load()) {
    std::lock_guard<std::mutex> lock(result_mtx_);
    return result_;
  }

  // Join a finished-but-unlocked worker before relaunching, so std::thread doesn't accumulate.
  if (worker_.joinable() && !search_running_.load()) {
    worker_.join();
  }

  if (!search_running_.exchange(true)) {
    worker_ = std::thread(&BnbVoxelRelocalization::workerMain, this);
  }

  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Worker thread
// ---------------------------------------------------------------------------
void BnbVoxelRelocalization::workerMain()
{
  if (!pyramid_built_ && !pyramid_failed_) {
    if (!buildPyramid()) {
      // Either stop_requested_ fired mid-build, or the map has no usable clouds
      // (pyramid_failed_ is set in the latter case so we never retry).
      search_running_ = false;
      return;
    }
    pyramid_built_ = true;
    if (publish_debug_grid_) publishDebugGrid();
  }

  if (pyramid_failed_ || stop_requested_.load()) {
    search_running_ = false;
    return;
  }

  std::vector<Eigen::Vector3d> query;
  {
    std::lock_guard<std::mutex> lock(scan_lock_);
    query = latest_query_;
  }

  // TEMPORARY diagnostic switch: when set (with debug_probe_pose_ also set), substitute a
  // keyframe self-query for the live scan and run it through the FULL production search path --
  // corridor roots, prefilter, branch-and-bound, NMS, GICP -- exactly like a real search. This is
  // deliberately different from debug_self_test_ above, which only scores a self-query in
  // isolation via searchPoses()'s own scoring sweeps: that verifies the SCORER can tell the true
  // pose apart from the rest, but says nothing about whether the SEARCH MACHINERY built on top of
  // it -- corridor roots sized by search_corridor_/z_margin_, the root prefilter, the
  // branch-and-bound node budget, NMS, the GICP acceptance gates -- can actually find a peak that
  // is definitely present. Substituting the query here, immediately before searchPoses(), routes
  // it through every one of those stages unmodified, so a failure here with debug_self_test_'s C1
  // passing cleanly points at the search machinery, not the scorer.
  if (debug_use_self_query_ && debug_probe_pose_.size() >= 4) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
    std::vector<Eigen::Vector3d> self_query;
    Eigen::Vector3d kf_t = Eigen::Vector3d::Zero();
    double kf_yaw = 0.0;
    int kf_idx = -1;
    if (buildSelfQuery(probe_t, self_query, kf_t, kf_yaw, kf_idx)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] debug_use_self_query: substituting keyframe %d self-query (n=%zu) at pose (%.2f,%.2f,%.2f) "
        "yaw=%.1fdeg for the live scan",
        name_.c_str(), kf_idx, self_query.size(), kf_t.x(), kf_t.y(), kf_t.z(), kf_yaw * 180.0 / M_PI);
      query = std::move(self_query);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "[%s] debug_use_self_query: buildSelfQuery() found no usable keyframe/cloud near the probe pose, "
        "falling back to the live scan",
        name_.c_str());
    }
  }

  if (query.empty()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "[%s] waiting for a live scan to search with", name_.c_str());
    search_running_ = false;
    return;
  }

  auto hypotheses = searchPoses(query);
  if (stop_requested_.load()) {
    search_running_ = false;
    return;
  }

  auto result = gicpPolish(hypotheses);
  if (result.has_value()) {
    {
      std::lock_guard<std::mutex> lock(result_mtx_);
      result_ = result;
    }
    result_ready_ = true;
    // Free search memory now: the plugin is never polled again after a successful lock. See
    // releasePyramidMemory() for why this is releaseMemory() (which also malloc_trim()s under
    // glibc) rather than clear() -- the user's memory-recovery requirement is that this is
    // actually recoverable, not merely dropped into the allocator's arena.
    releasePyramidMemory("lock");
    roots_.clear();
    roots_.shrink_to_fit();
    root_headings_.clear();
    root_headings_.shrink_to_fit();
    prefilter_candidates_.clear();
    prefilter_candidates_.shrink_to_fit();
    return;
  }

  // No acceptable candidate this round -- let a later tick relaunch with a fresher scan.
  search_running_ = false;
}

bool BnbVoxelRelocalization::buildPyramid()
{
  auto key_list = map_manager_->getKeyList();
  poses6d_ = map_manager_->getKeyPoses6D();  // deep copy, cached once

  eidos::reloc::VoxelPyramid::Config cfg;
  cfg.min_voxel_size = min_voxel_size_;
  cfg.num_levels = num_levels_;
  // Height filtering happens in the BODY frame (see insertKeyframeCloud and the query builder),
  // not here. A map-frame clamp is not comparable between map and query and cannot remove ground
  // on a map whose elevation varies, so the pyramid's own clamp stays disabled.
  cfg.max_height = 0.0;
  cfg.max_voxels_per_level = max_voxels_per_level_;
  // Under score_mode=distance_field the free-space channel is redundant (see onInitialize()'s
  // "inactive" log) and is skipped entirely here, not merely built and ignored -- it would be both
  // wasted build time and wasted memory. build_free_space_this_run_ (worker-owned state) is what
  // insertKeyframeCloud() actually checks below, rather than use_free_space_ directly, so the
  // raycast loop itself is skipped rather than just made a pyramid-side no-op.
  build_free_space_this_run_ = use_free_space_ && score_mode_ == eidos::reloc::ScoreMode::Occupancy;
  // The free-space band reuses min_height_/max_height_ verbatim -- the SAME body-frame band
  // applied to occupancy insertion below and to the live query in lidarCallback() -- so the three
  // can never drift out of sync with each other.
  cfg.build_free_space = build_free_space_this_run_;
  cfg.free_max_range = free_max_range_;
  cfg.free_end_margin = free_end_margin_;
  cfg.free_min_height = min_height_;
  cfg.free_max_height = max_height_;
  cfg.free_clear_near_occupied = free_clear_near_occupied_;
  cfg.max_free_voxels = max_free_voxels_;
  // Distance-field scoring config -- see the class doc on score_mode_ for the measurement that
  // motivated defaulting to distance_field. df_sigma_/df_truncation_voxels_ are consumed by the
  // pyramid's max-dilation build in finalize(); max_score_voxels_ guards the resulting grid size,
  // with the pyramid falling back to occupancy-equivalent (empty score grids) on overflow -- see
  // the fallback detection right after pyramid_.finalize() below.
  cfg.score_mode = score_mode_;
  cfg.df_sigma = df_sigma_;
  cfg.df_truncation_voxels = df_truncation_voxels_;
  cfg.max_score_voxels = max_score_voxels_;
  pyramid_.beginInsert(cfg);

  // Free-space rays originate at the LiDAR, not base_link: eidos's configured base_link is
  // actually base_footprint, which sits at ground level, while the LiDAR sits ~2.1 m above it. A
  // ray cast from ground level would spend most of its length below the min_height band and would
  // under-populate free space badly. Prefer the resolved base_link<-lidar TF (translation only,
  // exactly where the sensor sits); fall back to free_ray_origin_height_ above base_link if the TF
  // has not resolved yet (lidarCallback() runs on a different thread and may not have fired before
  // the worker reaches here on a fresh activation).
  const bool origin_from_tf = has_lidar_tf_;
  const Eigen::Vector3d sensor_offset_body =
    origin_from_tf ? T_base_lidar_.translation() : Eigen::Vector3d(0.0, 0.0, free_ray_origin_height_);
  if (build_free_space_this_run_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] free-space ray origin: %s, offset=(%.2f,%.2f,%.2f)",
      name_.c_str(),
      origin_from_tf ? "resolved base_link<-lidar TF" : "free_ray_origin_height parameter",
      sensor_offset_body.x(),
      sensor_offset_body.y(),
      sensor_offset_body.z());
  }

  trajectory_.clear();
  std::size_t rasterized_keyframes = 0;

  // The sibling "/gicp_cloud" slot is derived whenever the configured key ends in "/cloud". It is
  // tried ahead of the configured key when prefer_downsampled_source_ is set (default), because it
  // is already voxel-downsampled to ~0.5 m -- which loses essentially nothing at a 1.0 m finest
  // voxel and is far cheaper to rasterize than a full-resolution "*/cloud" -- and is otherwise kept
  // as the fallback of last resort, exactly as before.
  const std::string cloud_suffix = "/cloud";
  const bool has_fallback = pointcloud_from_.size() >= cloud_suffix.size() &&
    pointcloud_from_.compare(pointcloud_from_.size() - cloud_suffix.size(), cloud_suffix.size(), cloud_suffix) == 0;
  const std::string fallback_key =
    has_fallback ? pointcloud_from_.substr(0, pointcloud_from_.size() - cloud_suffix.size()) + "/gicp_cloud"
                 : std::string();

  std::unordered_map<std::string, std::size_t> source_counts;

  for (gtsam::Key key : key_list) {
    if (stop_requested_.load()) return false;
    if (!map_manager_->isPriorMapKey(key)) continue;

    int idx = map_manager_->getCloudIndex(key);
    if (idx < 0 || static_cast<std::size_t>(idx) >= poses6d_->points.size()) continue;

    Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(idx)]);
    Eigen::Isometry3d T;
    T.matrix() = world_t.matrix().cast<double>();

    bool inserted_any = false;
    bool tried_fallback = false;
    std::string used_key;

    if (has_fallback && prefer_downsampled_source_) {
      tried_fallback = true;
      if (insertKeyframeCloud(key, fallback_key, T, sensor_offset_body)) {
        inserted_any = true;
        used_key = fallback_key;
      }
    }
    if (!inserted_any && insertKeyframeCloud(key, pointcloud_from_, T, sensor_offset_body)) {
      inserted_any = true;
      used_key = pointcloud_from_;
    }
    if (!inserted_any && has_fallback && !tried_fallback && insertKeyframeCloud(key, fallback_key, T, sensor_offset_body)) {
      inserted_any = true;
      used_key = fallback_key;
    }

    if (!inserted_any) continue;

    ++source_counts[used_key];
    TrajectoryEntry entry;
    entry.position = T.translation();
    entry.cloud_index = idx;
    entry.key = key;
    // Map-frame heading of this keyframe, for the fine root prefilter's heading window (see
    // buildRoots()/prefilterRoots()). Same gtsam convention verified numerically in
    // buildSelfQuery() (Rz(yaw) * rotYX(pitch, roll), matched to ~5e-8) -- Rot3::yaw() alone is
    // exactly the third component of that decomposition, so no separate check is needed here.
    entry.yaw = gtsam::Rot3(T.rotation()).yaw();
    trajectory_.push_back(entry);
    ++rasterized_keyframes;
  }

  for (const auto & [key, count] : source_counts) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] %zu keyframes rasterized from source '%s'", name_.c_str(), count, key.c_str());
  }

  if (rasterized_keyframes == 0 || pyramid_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] pyramid build failed: no usable prior-map clouds under '%s' (or fallback '%s') -- the map has no "
      "usable clouds for relocalization",
      name_.c_str(),
      pointcloud_from_.c_str(),
      has_fallback ? fallback_key.c_str() : "<none>");
    pyramid_failed_ = true;
    return false;
  }

  pyramid_.finalize();

  // Determine the ACTUALLY built score mode. score_mode_ is what was requested; the pyramid falls
  // back to Occupancy internally if max_score_voxels_ is exceeded while building the distance
  // field, exactly like freeSpaceAbandoned() detects the analogous free-space overflow --
  // effectiveScoreMode() is the authoritative accessor for this (distanceFieldAbandoned() is the
  // WARN condition). Everything downstream (SearchConfig::score_mode and every probe diagnostic)
  // reads active_score_mode_, never score_mode_, so a silent fallback can never be misreported as
  // the mode that was requested.
  active_score_mode_ = pyramid_.effectiveScoreMode();
  if (pyramid_.distanceFieldAbandoned()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] distance-field grid exceeded max_score_voxels (%zu) during build; falling back to "
      "occupancy scoring for this session",
      name_.c_str(),
      max_score_voxels_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] pyramid: %zu prior keyframes rasterized, %.2f MB, score_mode=%s",
    name_.c_str(),
    rasterized_keyframes,
    static_cast<double>(pyramid_.memoryBytes()) / 1048576.0,
    scoreModeLabel(active_score_mode_));
  for (int l = 0; l < pyramid_.numLevels(); ++l) {
    const auto & lvl = pyramid_.level(l);
    if (active_score_mode_ == eidos::reloc::ScoreMode::DistanceField) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s]   level %d: res=%.2fm voxels=%zu score_voxels=%zu score_mb=%.2f",
        name_.c_str(),
        l,
        lvl.resolution,
        lvl.voxels.size(),
        pyramid_.scoreVoxelCount(l),
        static_cast<double>(lvl.scores.memoryBytes()) / 1048576.0);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s]   level %d: res=%.2fm voxels=%zu free_voxels=%zu",
        name_.c_str(),
        l,
        lvl.resolution,
        lvl.voxels.size(),
        pyramid_.freeVoxelCount(l));
    }
    if (pyramid_.dilationSkipped(l)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[%s]   level %d exceeded max_voxels_per_level, probing neighbourhood on the fly",
        name_.c_str(),
        l);
    }
  }
  // Expected and sound: the height band makes the free volume only a few metres thick, so the
  // "all 8 children free" test that builds coarser free levels empties out at level >= 2 (a fully
  // free 4 m cell needs 64 stacked free 1 m voxels). Free space is therefore only active at levels
  // 0-1 -- the coarse bound stays as loose as it always was, and the discrimination happens at the
  // leaf and in the acceptance gate, which is exactly where the uniqueness ratio is computed.
  if (build_free_space_this_run_ && pyramid_.freeSpaceAbandoned()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] free space abandoned: max_free_voxels (%zu) exceeded during build; falling back to the "
      "binary occupancy score for this session",
      name_.c_str(),
      max_free_voxels_);
  }
  if (pyramid_.outOfRangePoints() > 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] %zu points dropped: voxel index exceeded the packing range",
      name_.c_str(),
      pyramid_.outOfRangePoints());
  }

  if (stop_requested_.load()) return false;
  buildRoots();
  return true;
}

bool BnbVoxelRelocalization::insertKeyframeCloud(
  gtsam::Key key, const std::string & data_key, const Eigen::Isometry3d & world_t,
  const Eigen::Vector3d & sensor_offset_body)
{
  const Eigen::Vector3d origin_world = world_t * sensor_offset_body;

  auto pcl_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(key, data_key);
  if (pcl_opt.has_value() && *pcl_opt && !(*pcl_opt)->empty()) {
    const std::size_t n = (*pcl_opt)->points.size();
    const std::size_t stride = raycastStride(n, free_rays_per_keyframe_);
    for (std::size_t i = 0; i < n; ++i) {
      const auto & pt = (*pcl_opt)->points[i];
      if (inHeightBand(pt.z)) pyramid_.insert(world_t * Eigen::Vector3d(pt.x, pt.y, pt.z));
      // Free-space raycasting rides the SAME loop as the occupancy insertion above -- insertRay()
      // is not thread-safe, and a second pass over the cloud would double the iteration cost for
      // no benefit. The endpoint is NOT height-band filtered here: insertRay() applies the band
      // per-voxel along the ray (relative to the ray origin), so a ray toward an out-of-band
      // endpoint still marks the in-band portion of its path free.
      if (build_free_space_this_run_ && (i % stride == 0)) {
        if (stop_requested_.load()) break;
        pyramid_.insertRay(origin_world, world_t * Eigen::Vector3d(pt.x, pt.y, pt.z));
      }
    }
    return true;
  }

  auto gicp_opt = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(key, data_key);
  if (gicp_opt.has_value() && *gicp_opt && !(*gicp_opt)->empty()) {
    const std::size_t n = (*gicp_opt)->size();
    const std::size_t stride = raycastStride(n, free_rays_per_keyframe_);
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d p = (*gicp_opt)->point(i).head<3>();
      if (inHeightBand(p.z())) pyramid_.insert(world_t * p);
      if (build_free_space_this_run_ && (i % stride == 0)) {
        if (stop_requested_.load()) break;
        pyramid_.insertRay(origin_world, world_t * p);
      }
    }
    return true;
  }

  return false;
}

void BnbVoxelRelocalization::buildRoots()
{
  roots_.clear();
  root_headings_.clear();  // Kept parallel to roots_ -- see its doc comment in the header.
  if (pyramid_.empty()) return;

  const int coarsest = pyramid_.numLevels() - 1;
  const auto & lvl = pyramid_.level(coarsest);
  const double res = lvl.resolution;
  const double inv_res = lvl.inv_resolution;
  const int64_t corridor_cells = static_cast<int64_t>(std::ceil(search_corridor_ / res));
  const int64_t z_cells = static_cast<int64_t>(std::ceil(z_margin_ / res));

  eidos::reloc::VoxelSet seen;

  for (const auto & entry : trajectory_) {
    if (stop_requested_.load()) return;

    const int64_t cx = eidos::reloc::voxelIndex(entry.position.x(), inv_res);
    const int64_t cy = eidos::reloc::voxelIndex(entry.position.y(), inv_res);
    const int64_t cz = eidos::reloc::voxelIndex(entry.position.z(), inv_res);

    for (int64_t dx = -corridor_cells; dx <= corridor_cells; ++dx) {
      for (int64_t dy = -corridor_cells; dy <= corridor_cells; ++dy) {
        // Root cells live at the COARSEST pyramid level (8 m by default), so testing the cell
        // CENTRE against search_corridor_ is wrong: a cell can be kept or dropped based on where
        // its centre happens to fall even though poses much closer than search_corridor_ sit
        // inside that same cell -- up to res*sqrt(3)/2 (~6.9 m at 8 m resolution) closer than the
        // centre-to-trajectory-point distance being tested. This is not a theoretical concern --
        // it measurably excluded the true pose on the
        // validation bag: the vehicle starts 19.88 m from the nearest keyframe, but the cell
        // containing that start point has its CENTRE 23.1 m away, so at the old 20 m default the
        // true cell was silently never even added as a root. Use a CONSERVATIVE cell-extent test
        // instead: clamp the trajectory point into the cell's [lo, lo+res] box on each axis and
        // measure to that clamped point, i.e. to the NEAREST point of the cell. This keeps the
        // cell whenever ANY pose inside it could be within the corridor, which is what the search
        // actually needs -- the cell-centre test above silently dropped boundary cells that still
        // contain in-corridor poses.
        const double lo_x = static_cast<double>(cx + dx) * res;
        const double lo_y = static_cast<double>(cy + dy) * res;
        const double nx = std::clamp(entry.position.x(), lo_x, lo_x + res);
        const double ny = std::clamp(entry.position.y(), lo_y, lo_y + res);
        if (std::hypot(nx - entry.position.x(), ny - entry.position.y()) > search_corridor_) continue;

        for (int64_t dz = -z_cells; dz <= z_cells; ++dz) {
          // Same clamp-to-nearest-point test on z, against z_margin_.
          const double lo_z = static_cast<double>(cz + dz) * res;
          const double nz = std::clamp(entry.position.z(), lo_z, lo_z + res);
          if (std::abs(nz - entry.position.z()) > z_margin_) continue;

          const int64_t packed = eidos::reloc::packVoxel(cx + dx, cy + dy, cz + dz);
          if (seen.insert(packed).second) {
            roots_.push_back(eidos::reloc::RootCell{cx + dx, cy + dy, cz + dz});
            // Heading of the FIRST trajectory entry that admits this cell (VoxelSet::insert()'s
            // .second above is only true once per cell, so this runs exactly once per root, in
            // lockstep with the push_back above -- root_headings_ stays index-parallel to roots_).
            root_headings_.push_back(static_cast<float>(entry.yaw));
          }
        }
      }
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] built %zu corridor roots at coarsest resolution %.2f m",
    name_.c_str(),
    roots_.size(),
    res);
}

void BnbVoxelRelocalization::publishDebugGrid()
{
  if (!debug_grid_pub_ || pyramid_.empty()) return;

  // Level 0 is exact (undilated), which best matches the underlying map for visual comparison
  // against slam/visualization/map in RViz.
  const auto & lvl = pyramid_.level(0);
  const double res = lvl.resolution;

  int64_t min_ix = std::numeric_limits<int64_t>::max();
  int64_t max_ix = std::numeric_limits<int64_t>::min();
  int64_t min_iy = std::numeric_limits<int64_t>::max();
  int64_t max_iy = std::numeric_limits<int64_t>::min();

  for (int64_t key : lvl.voxels) {
    int64_t ix, iy, iz;
    eidos::reloc::unpackVoxel(key, ix, iy, iz);
    min_ix = std::min(min_ix, ix);
    max_ix = std::max(max_ix, ix);
    min_iy = std::min(min_iy, iy);
    max_iy = std::max(max_iy, iy);
  }
  if (min_ix > max_ix || min_iy > max_iy) return;

  const int64_t width = max_ix - min_ix + 1;
  const int64_t height = max_iy - min_iy + 1;
  constexpr int64_t kMaxGridDim = 100000;
  if (width <= 0 || height <= 0 || width > kMaxGridDim || height > kMaxGridDim) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] debug grid extent too large to publish (%ldx%ld cells), skipping",
      name_.c_str(),
      static_cast<long>(width),   // NOLINT(runtime/int)
      static_cast<long>(height));  // NOLINT(runtime/int)
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = map_frame_;
  grid.header.stamp = node_->now();
  grid.info.resolution = static_cast<float>(res);
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.origin.position.x = static_cast<double>(min_ix) * res;
  grid.info.origin.position.y = static_cast<double>(min_iy) * res;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);

  for (int64_t key : lvl.voxels) {
    int64_t ix, iy, iz;
    eidos::reloc::unpackVoxel(key, ix, iy, iz);
    const int64_t gx = ix - min_ix;
    const int64_t gy = iy - min_iy;
    grid.data[static_cast<std::size_t>(gy * width + gx)] = 100;
  }

  if (debug_grid_pub_->is_activated()) {
    debug_grid_pub_->publish(grid);
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] published debug voxel grid: %ldx%ld cells @ %.2fm",
      name_.c_str(),
      static_cast<long>(width),   // NOLINT(runtime/int)
      static_cast<long>(height),  // NOLINT(runtime/int)
      res);
  }
}

// ---------------------------------------------------------------------------
// Phase C — branch-and-bound search over a roll/pitch offset grid
// ---------------------------------------------------------------------------
std::vector<BnbVoxelRelocalization::FineRootScore> BnbVoxelRelocalization::scoreRootsFine(
  const std::vector<Eigen::Vector3d> & sub_query) const
{
  const int coarsest_level = pyramid_.numLevels() - 1;
  const double coarse_res = pyramid_.level(coarsest_level).resolution;

  // Evenly divide [lo, lo+res] into ceil-ish n = max(1, round(res/step)) sub-cells and sample
  // their CENTRES -- this both honours the requested step size and, by construction, always sits
  // strictly inside the cell (never on a face), regardless of how step relates to res.
  auto sampleAxis = [](double lo, double res, double step) {
    const int n = std::max(1, static_cast<int>(std::round(res / std::max(step, 1e-6))));
    const double sub = res / static_cast<double>(n);
    std::vector<double> vals;
    vals.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) vals.push_back(lo + (static_cast<double>(i) + 0.5) * sub);
    return vals;
  };

  // Yaw offsets (relative to a root's own recorded heading) are identical for every root, so
  // compute them once outside the parallel loop rather than per root.
  std::vector<double> yaw_offsets_rad;
  {
    const double step_rad = std::max(prefilter_yaw_step_deg_, 1e-3) * M_PI / 180.0;
    if (use_heading_prior_) {
      const double tol_rad = heading_tolerance_deg_ * M_PI / 180.0;
      for (double off = -tol_rad; off <= tol_rad + 1e-9; off += step_rad) yaw_offsets_rad.push_back(off);
    } else {
      for (double a = 0.0; a < 2.0 * M_PI - 1e-9; a += step_rad) yaw_offsets_rad.push_back(a);
    }
    if (yaw_offsets_rad.empty()) yaw_offsets_rad.push_back(0.0);
  }

  std::vector<FineRootScore> scored(roots_.size());
  // Parallel over roots, exactly like the coarse path below and every other per-root loop in this
  // file: each iteration writes only its own slot and the pyramid is read-only, so no
  // synchronisation is required.
#pragma omp parallel for schedule(static) num_threads(num_threads_)
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(roots_.size()); ++i) {
    const std::size_t idx = static_cast<std::size_t>(i);
    // OpenMP forbids breaking out of a parallel for, so a stop request marks the remaining roots
    // unusable (score = -1) instead; the caller aborts right after.
    if (stop_requested_.load()) {
      scored[idx] = FineRootScore{-1, idx, Eigen::Vector3d::Zero(), 0.0};
      continue;
    }

    const auto & root = roots_[idx];
    const double lo_x = static_cast<double>(root.ix) * coarse_res;
    const double lo_y = static_cast<double>(root.iy) * coarse_res;
    const double lo_z = static_cast<double>(root.iz) * coarse_res;
    const auto xs = sampleAxis(lo_x, coarse_res, prefilter_xy_step_);
    const auto ys = sampleAxis(lo_y, coarse_res, prefilter_xy_step_);
    const auto zs = sampleAxis(lo_z, coarse_res, prefilter_z_step_);
    const double heading = idx < root_headings_.size() ? static_cast<double>(root_headings_[idx]) : 0.0;

    int best_score = -1;
    Eigen::Vector3d best_pos = Eigen::Vector3d::Zero();
    double best_yaw = 0.0;
    for (double x : xs) {
      for (double y : ys) {
        for (double z : zs) {
          const Eigen::Vector3d pos(x, y, z);
          for (double off : yaw_offsets_rad) {
            // hit_weight_ here, NOT the hard-coded 3 the coarse path below passes -- this is the
            // correction the fine path makes to an existing inconsistency with the configured
            // weight (see the coarse branch's own comment on that literal). Level 0 (not
            // coarsest_level): the coarse level is exactly what saturates -- see this function's
            // header doc comment for the measurement.
            const int score = eidos::reloc::scorePoseAtLevel(
              pyramid_, sub_query, pos, off + heading, 0, hit_weight_, 0, nullptr, active_score_mode_);
            if (score > best_score) {
              best_score = score;
              best_pos = pos;
              best_yaw = off + heading;
            }
            if (use_heading_prior_ && allow_reverse_heading_) {
              const int score_rev = eidos::reloc::scorePoseAtLevel(
                pyramid_, sub_query, pos, off + heading + M_PI, 0, hit_weight_, 0, nullptr, active_score_mode_);
              if (score_rev > best_score) {
                best_score = score_rev;
                best_pos = pos;
                best_yaw = off + heading + M_PI;
              }
            }
          }
        }
      }
    }
    scored[idx] = FineRootScore{best_score, idx, best_pos, best_yaw};
  }
  return scored;
}

std::vector<eidos::reloc::RootCell> BnbVoxelRelocalization::prefilterRoots(
  const std::vector<Eigen::Vector3d> & rotated_query, double dr, double dp)
{
  // WHY THIS FUNCTION HAS TWO PATHS -- MEASURED ROOT CAUSE of a 281 m mislocalization on the
  // validation bag: the ORIGINAL prefilter (still available as the `prefilter_fine_ == false`
  // path below) ranks corridor roots by scoring them at the COARSEST pyramid level with a
  // `root_prefilter_points_`-point subsample. Under the distance-field score the ceiling is
  // `root_prefilter_points_ * 255` (128 * 255 = 32640 at the default), and the coarsest level's
  // 26-neighbourhood dilation covers roughly 2.5x the cell size, so essentially every root near
  // the road saturates at that ceiling: 11611 of 11613 roots tied at the exact maximum on the
  // validation bag. `std::nth_element` (the old selection algorithm) makes NO guarantee about
  // which tied elements land in the kept prefix, so the kept set was effectively arbitrary among
  // the ties and the true root survived with only about a 2% chance. The branch-and-bound BOUND
  // itself is not the bug -- it is sound everywhere -- it is simply as uninformative as the
  // prefilter at coarse levels, for exactly the same saturation reason, so a root that slips
  // through the prefilter gets no help from BnB either.
  //
  // Level 0 DOES discriminate: for a query known to be good, the reference pose scored 94623 at
  // its leaf cell centre (99309 at the exact pose), while the best WRONG pose anywhere in the map
  // scored 77885 -- a 1.22x margin. So instead of ranking roots by a coarse score that cannot
  // tell them apart, the fine path (default) ranks them by a heading-constrained LEVEL-0 scan
  // inside each root's cell -- see scoreRootsFine().
  if (root_prefilter_keep_ <= 0 || root_prefilter_keep_ >= static_cast<int>(roots_.size())) {
    return roots_;  // Prefilter disabled: exactly recover the un-filtered behaviour.
  }

  // Strided subsample, not a contiguous prefix -- a prefix would be spatially biased because the
  // scan is ordered. Shared by both paths below.
  const std::size_t n = rotated_query.size();
  const std::size_t target = std::min(n, static_cast<std::size_t>(std::max(root_prefilter_points_, 1)));
  std::vector<Eigen::Vector3d> sub_query;
  sub_query.reserve(target);
  if (target > 0 && n > 0) {
    const double step = static_cast<double>(n) / static_cast<double>(target);
    for (std::size_t k = 0; k < target; ++k) {
      std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
      if (idx >= n) idx = n - 1;
      sub_query.push_back(rotated_query[idx]);
    }
  }

  if (!prefilter_fine_) {
    // -------------------------------------------------------------------------------------------
    // ORIGINAL coarse-level ranking, UNCHANGED, kept so `prefilter_fine_: false` reproduces
    // today's behaviour exactly for A/B testing against the fine path above.
    // -------------------------------------------------------------------------------------------
    // Use the same discretisation branchAndBound() will compute internally for this rotated
    // query, so the coarsest-level yaw bins scored here line up exactly with the ones it will
    // seed from.
    const eidos::reloc::YawDiscretization yaw_disc =
      eidos::reloc::YawDiscretization::compute(rotated_query, pyramid_);
    if (yaw_disc.max_range <= 0.0) return roots_;  // degenerate query, let branchAndBound() handle it.

    const int coarsest_level = pyramid_.numLevels() - 1;
    const double coarse_res = pyramid_.level(coarsest_level).resolution;
    const int64_t n_coarse_bins = yaw_disc.numBins(coarsest_level);

    // (best coarse score over all yaw bins, index into roots_)
    // Scored in parallel over roots. Each iteration writes only its own slot and the
    // pyramid is read-only here, so no synchronisation is required. Measurement showed
    // this loop dominated prefilter cost when serial (20 s over a 3337-root corridor),
    // while the main search was already parallel.
    std::vector<std::pair<int, std::size_t>> scored(roots_.size());
#pragma omp parallel for schedule(static) num_threads(num_threads_)
    for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(roots_.size()); ++i) {
      const std::size_t idx = static_cast<std::size_t>(i);
      // OpenMP forbids breaking out of a parallel for, so a stop request marks the
      // remaining roots unusable (-1) instead; the caller aborts right after.
      if (stop_requested_.load()) {
        scored[idx] = {-1, idx};
        continue;
      }

      const auto & root = roots_[idx];
      const Eigen::Vector3d centre = eidos::reloc::bnbCellCentre(root.ix, root.iy, root.iz, coarse_res);
      int best = -1;
      for (int64_t k = 0; k < n_coarse_bins; ++k) {
        const double yaw = yaw_disc.binCentre(coarsest_level, k);
        // Trailing score_mode must be active_score_mode_, not the DistanceField default: under
        // score_mode=occupancy the pyramid never built its score grids, so scoring against the
        // default here would silently read all-zero cells and defeat the prefilter.
        const int score = eidos::reloc::scorePoseAtLevel(
          pyramid_, sub_query, centre, yaw, coarsest_level, 3, 0, nullptr, active_score_mode_);
        if (score > best) best = score;
      }
      scored[idx] = {best, idx};
    }

    const std::size_t keep = std::min(static_cast<std::size_t>(root_prefilter_keep_), scored.size());
    std::nth_element(
      scored.begin(), scored.begin() + static_cast<std::ptrdiff_t>(keep), scored.end(),
      [](const std::pair<int, std::size_t> & a, const std::pair<int, std::size_t> & b) {
        return a.first > b.first;
      });
    scored.resize(keep);

    std::vector<eidos::reloc::RootCell> kept;
    kept.reserve(keep);
    for (const auto & [score, idx] : scored) kept.push_back(roots_[idx]);
    return kept;
  }

  // ---------------------------------------------------------------------------------------------
  // FINE, heading-constrained level-0 ranking (default). See scoreRootsFine() for the per-root
  // scan and this function's header comment above for the saturation measurement that motivates
  // it.
  // ---------------------------------------------------------------------------------------------
  auto scored = scoreRootsFine(sub_query);

  const std::size_t keep = std::min(static_cast<std::size_t>(root_prefilter_keep_), scored.size());
  // std::partial_sort, not std::nth_element: nth_element's unspecified handling of tied elements
  // is PRECISELY what discarded the true root in the coarse path above, so the fine path uses a
  // fully-ordered, deterministic selection instead. The comparator also breaks ties on `idx`
  // (ascending), giving a strict weak ordering with NO ties at all -- so which roots land in the
  // kept prefix is reproducible for identical inputs regardless of standard-library
  // implementation, not merely "more ordered than nth_element".
  std::partial_sort(
    scored.begin(), scored.begin() + static_cast<std::ptrdiff_t>(keep), scored.end(),
    [](const FineRootScore & a, const FineRootScore & b) {
      return a.score != b.score ? a.score > b.score : a.idx < b.idx;
    });

  // Emit the argmax poses of the top-ranked roots as REAL candidates, not just a kept-root list:
  // because branch-and-bound's own coarse-level bound saturates at the same ceiling that broke
  // the old prefilter (see this function's header comment), a root surviving into BnB gets no
  // guidance from the bound either. Materialising the fine scan's own best pose per root means the
  // correct pose can still surface even if BnB's search over that root's subtree does not
  // rediscover it. Capped at min(keep, 4 * num_gicp_candidates_): only the GICP polish stage can
  // use more than a handful of hypotheses productively, so scoring/emitting more than that here
  // would be wasted work.
  const std::size_t n_candidates =
    std::min(keep, static_cast<std::size_t>(std::max(4 * num_gicp_candidates_, 0)));
  for (std::size_t i = 0; i < n_candidates; ++i) {
    const FineRootScore & fr = scored[i];
    if (fr.score < 0) continue;  // stop_requested_ fired mid-scan; skip the unusable entry.
    // Re-score the argmax pose against the FULL (non-subsampled) rotated query, at level 0, via
    // the same scoreBreakdownAtLevel() branchAndBound()'s own leafHypothesis() uses -- so a
    // prefilter candidate's score sits on the same footing as a BnB-produced hypothesis when the
    // two are merged and NMS'd together in searchPoses().
    const eidos::reloc::ScoreBreakdown breakdown = eidos::reloc::scoreBreakdownAtLevel(
      pyramid_, rotated_query, fr.pos, fr.yaw, 0, hit_weight_, active_score_mode_);
    eidos::reloc::Hypothesis hyp;
    hyp.translation = fr.pos;
    hyp.yaw = fr.yaw;
    hyp.score = breakdown.raw;
    hyp.hits = breakdown.hits;
    hyp.hit_fraction =
      static_cast<double>(breakdown.hits) / static_cast<double>(std::max<std::size_t>(1, rotated_query.size()));
    hyp.normalized = breakdown.max_possible > 0
                       ? static_cast<double>(breakdown.raw) / static_cast<double>(breakdown.max_possible)
                       : 0.0;
    prefilter_candidates_.push_back(ScoredHypothesis{hyp, dr, dp});
  }

  std::vector<eidos::reloc::RootCell> kept;
  kept.reserve(keep);
  for (std::size_t i = 0; i < keep; ++i) kept.push_back(roots_[scored[i].idx]);
  return kept;
}

// ---------------------------------------------------------------------------
// Self-query builder -- a query drawn from a prior-map keyframe's own cloud, exactly registered
// to the map by construction. Shared by debug_self_test_'s C1/C2/C3 scoring-only diagnostics
// (searchPoses()) and by debug_use_self_query_'s full-pipeline substitution (workerMain()). See
// the doc comment in the header for the full contract.
// ---------------------------------------------------------------------------
bool BnbVoxelRelocalization::buildSelfQuery(
  const Eigen::Vector3d & near_position, std::vector<Eigen::Vector3d> & query_out,
  Eigen::Vector3d & kf_translation_out, double & kf_yaw_out, int & kf_index_out)
{
  query_out.clear();
  kf_translation_out = Eigen::Vector3d::Zero();
  kf_yaw_out = 0.0;
  kf_index_out = -1;

  // -----------------------------------------------------------------------------------------
  // Select the reference keyframe: the prior-map keyframe whose position is closest to
  // `near_position`. Same iteration pattern buildPyramid() uses.
  // -----------------------------------------------------------------------------------------
  gtsam::Key kf_key = 0;
  int kf_idx = -1;
  Eigen::Isometry3d T_kf = Eigen::Isometry3d::Identity();
  double best_d = std::numeric_limits<double>::infinity();
  bool have_kf = false;
  {
    auto key_list = map_manager_->getKeyList();
    for (gtsam::Key key : key_list) {
      if (!map_manager_->isPriorMapKey(key)) continue;
      int idx = map_manager_->getCloudIndex(key);
      if (idx < 0 || static_cast<std::size_t>(idx) >= poses6d_->points.size()) continue;
      Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(idx)]);
      Eigen::Isometry3d T;
      T.matrix() = world_t.matrix().cast<double>();
      const double d = (T.translation() - near_position).norm();
      if (d < best_d) {
        best_d = d;
        T_kf = T;
        kf_key = key;
        kf_idx = idx;
        have_kf = true;
      }
    }
  }
  if (!have_kf) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] buildSelfQuery: no prior-map keyframe found near (%.2f,%.2f,%.2f)", name_.c_str(),
      near_position.x(), near_position.y(), near_position.z());
    return false;
  }

  // Decompose T_kf's rotation into roll/pitch/yaw with gtsam's own RzRyRx convention
  // (Rot3::rpy() returns [roll, pitch, yaw] such that R == Rot3::Ypr(yaw, pitch, roll) ==
  // RzRyRx(roll, pitch, yaw) == Rz(yaw) * Ry(pitch) * Rx(roll) == Rz(yaw) * rotYX(pitch, roll)),
  // then VERIFY that identity numerically rather than trusting the derivation: if this plugin's
  // rotYX()/Rz(yaw) composition does not actually match gtsam's convention, every de-tilt in the
  // file -- not just this function -- is silently wrong.
  const gtsam::Vector3 rpy_kf = gtsam::Rot3(T_kf.rotation()).rpy();
  const double roll_kf = rpy_kf(0);
  const double pitch_kf = rpy_kf(1);
  const double yaw_kf = rpy_kf(2);
  {
    Eigen::Matrix3d rz_yaw;
    rz_yaw << std::cos(yaw_kf), -std::sin(yaw_kf), 0.0, std::sin(yaw_kf), std::cos(yaw_kf), 0.0, 0.0, 0.0, 1.0;
    const Eigen::Matrix3d recon = rz_yaw * rotYX(pitch_kf, roll_kf);
    const double conv_err = (recon - T_kf.rotation()).cwiseAbs().maxCoeff();
    if (conv_err > 1e-6) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[%s] buildSelfQuery: rotation convention check failed for kf=%d (err=%.3e > 1e-6) -- every de-tilt in "
        "this plugin is suspect, not just this diagnostic",
        name_.c_str(), kf_idx, conv_err);
    }
  }

  // Retrieve the keyframe's own BODY-frame cloud. Same PCL-typed-then-small_gicp-typed
  // retrieval, and source-key selection, that insertKeyframeCloud()/buildPyramid() use. Returns
  // BODY-frame points -- not transformed by the keyframe's world pose, no height band applied.
  const std::string cloud_suffix = "/cloud";
  const bool has_fallback = pointcloud_from_.size() >= cloud_suffix.size() &&
    pointcloud_from_.compare(pointcloud_from_.size() - cloud_suffix.size(), cloud_suffix.size(), cloud_suffix) == 0;
  const std::string fallback_key = has_fallback
    ? pointcloud_from_.substr(0, pointcloud_from_.size() - cloud_suffix.size()) + "/gicp_cloud"
    : std::string();
  auto tryKey = [&](const std::string & data_key, std::vector<Eigen::Vector3d> & out) -> bool {
    auto pcl_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(kf_key, data_key);
    if (pcl_opt.has_value() && *pcl_opt && !(*pcl_opt)->empty()) {
      out.reserve(out.size() + (*pcl_opt)->points.size());
      for (const auto & pt : (*pcl_opt)->points) out.emplace_back(pt.x, pt.y, pt.z);
      return true;
    }
    auto gicp_opt = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(kf_key, data_key);
    if (gicp_opt.has_value() && *gicp_opt && !(*gicp_opt)->empty()) {
      out.reserve(out.size() + (*gicp_opt)->size());
      for (std::size_t i = 0; i < (*gicp_opt)->size(); ++i) {
        const Eigen::Vector3d p = (*gicp_opt)->point(i).head<3>();
        out.push_back(p);
      }
      return true;
    }
    return false;
  };
  std::vector<Eigen::Vector3d> kf_body;
  const bool retrieved = (has_fallback && prefer_downsampled_source_)
    ? (tryKey(fallback_key, kf_body) || tryKey(pointcloud_from_, kf_body))
    : (tryKey(pointcloud_from_, kf_body) || (has_fallback && tryKey(fallback_key, kf_body)));
  if (!retrieved || kf_body.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] buildSelfQuery: no usable cloud found for kf=%d", name_.c_str(), kf_idx);
    return false;
  }

  // Mirror the live query builder's transform chain (lidarCallback()): de-tilt by
  // rotYX(pitch, roll), drop points beyond max_query_range_ horizontally, keep only the
  // configured height band, then apply the identical strided downsample to
  // target_query_points_. The only difference from the live path is WHERE the points and the
  // roll/pitch come from -- a keyframe's own recorded cloud and pose, not a live scan and IMU.
  const Eigen::Matrix3d r_detilt = rotYX(pitch_kf, roll_kf);
  const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
  std::vector<Eigen::Vector3d> filtered;
  filtered.reserve(kf_body.size());
  for (const auto & p : kf_body) {
    const Eigen::Vector3d pd = r_detilt * p;
    if (max_range_sq > 0.0 && pd.head<2>().squaredNorm() > max_range_sq) continue;
    if (!inHeightBand(pd.z())) continue;
    filtered.push_back(pd);
  }
  if (target_query_points_ > 0 && static_cast<int>(filtered.size()) > target_query_points_) {
    const std::size_t target = static_cast<std::size_t>(target_query_points_);
    const std::size_t n = filtered.size();
    const double step = static_cast<double>(n) / static_cast<double>(target);
    query_out.reserve(target);
    for (std::size_t k = 0; k < target; ++k) {
      std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
      if (idx >= n) idx = n - 1;
      query_out.push_back(filtered[idx]);
    }
  } else {
    query_out = std::move(filtered);
  }

  kf_translation_out = T_kf.translation();
  kf_yaw_out = yaw_kf;
  kf_index_out = kf_idx;
  return true;
}

std::vector<BnbVoxelRelocalization::ScoredHypothesis> BnbVoxelRelocalization::searchPoses(
  const std::vector<Eigen::Vector3d> & query)
{
  std::vector<ScoredHypothesis> nms_result;
  if (query.empty() || roots_.empty()) return nms_result;

  std::vector<double> roll_offsets, pitch_offsets;
  if (rp_search_steps_ <= 1) {
    roll_offsets = {0.0};
    pitch_offsets = {0.0};
  } else {
    for (int i = 0; i < rp_search_steps_; ++i) {
      const double frac = static_cast<double>(i) / static_cast<double>(rp_search_steps_ - 1);
      const double t = -rp_search_range_ + (2.0 * rp_search_range_) * frac;
      roll_offsets.push_back(t);
      pitch_offsets.push_back(t);
    }
  }

  struct OffsetPair
  {
    double dr;
    double dp;
  };
  std::vector<OffsetPair> offsets;
  offsets.reserve(roll_offsets.size() * pitch_offsets.size());
  for (double dr : roll_offsets) {
    for (double dp : pitch_offsets) {
      offsets.push_back({dr, dp});
    }
  }

  // Per-offset preparation: rotate the query and cheaply prefilter the roots down to the most
  // promising ones (see prefilterRoots()), then split the survivors into num_threads_ chunks so
  // the main search loop below has (offset x chunk) tasks to parallelize over, not just offsets.
  struct OffsetPrep
  {
    double dr = 0.0;
    double dp = 0.0;
    std::vector<Eigen::Vector3d> rotated_query;
    std::vector<std::vector<eidos::reloc::RootCell>> root_chunks;
  };
  std::vector<OffsetPrep> preps(offsets.size());

  const std::size_t roots_before = roots_.size();
  std::size_t roots_after = roots_before;
  const auto t_prefilter_start = std::chrono::steady_clock::now();

  // Fresh per search: prefilterRoots()'s fine path (see its doc comment) appends the argmax pose
  // of each surviving root's local scan here, once per offset below; searchPoses() merges this
  // into the candidate list right before NMS, further down.
  prefilter_candidates_.clear();

  // Deliberately serial over offsets. prefilterRoots() parallelises internally over roots,
  // which is far finer-grained than this loop (thousands of roots vs. typically one offset),
  // and OpenMP serialises nested parallel regions by default -- so parallelising here would
  // silently disable that inner parallelism. Measured: with a pragma here the prefilter took
  // 21.8 s; without it the inner loop can actually use every core. This loop's wall time (timed
  // below) is also where the fine prefilter's level-0 scan cost shows up, so the "root prefilter:
  // N -> M roots, X ms" log line already reports it -- no separate timer is needed.
  for (int i = 0; i < static_cast<int>(offsets.size()); ++i) {
    if (stop_requested_.load()) continue;

    auto & prep = preps[static_cast<std::size_t>(i)];
    prep.dr = offsets[static_cast<std::size_t>(i)].dr;
    prep.dp = offsets[static_cast<std::size_t>(i)].dp;
    const Eigen::Matrix3d r_offset = rotYX(prep.dp, prep.dr);

    prep.rotated_query.reserve(query.size());
    for (const auto & q : query) prep.rotated_query.push_back(r_offset * q);

    auto kept_roots = prefilterRoots(prep.rotated_query, prep.dr, prep.dp);
    prep.root_chunks = chunkRoots(kept_roots, num_threads_);

    // -------------------------------------------------------------------
    // TRACE 9 -- prefilter kept-set: TRACE 4 (further below) recomputes a ranking independently of
    // prefilterRoots() itself -- it never reads the vector prefilterRoots() actually returned, only
    // where the reference root's score falls among every root's score. At the coarsest level the
    // prefilter's score saturates: it sums `root_prefilter_points_` per-point cell values capped at
    // 255 each under DistanceField, so a subsample that lands entirely on structure scores the exact
    // same maximum (root_prefilter_points_ * 255) regardless of exactly how well-aligned it is
    // beyond that point -- meaning a dense map can tie THOUSANDS of roots at that ceiling. TRACE 4's
    // "rank 1" in that regime only means "rank 1 among ties"; it says nothing about whether the
    // reference root is among the `root_prefilter_keep_` that std::nth_element() (see
    // prefilterRoots() above) actually chose to keep -- nth_element() makes NO guarantee about WHICH
    // elements tied at its partition value end up on which side of the partition. This checks the
    // ACTUAL returned `kept_roots` vector directly, and separately measures how saturated the tie
    // actually is, so the two failure modes ("reference root never made the cut" vs. "tie-breaking
    // is fine, something else is wrong") are told apart by data, not assumption.
    // -------------------------------------------------------------------
    if (debug_probe_pose_.size() >= 4) {
      const Eigen::Vector3d trace9_probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
      const int trace9_coarsest = pyramid_.numLevels() - 1;
      const auto & trace9_coarse_lvl = pyramid_.level(trace9_coarsest);
      // Same cell-centre convention as TRACE 2 / TRACE 8: the coarsest-level voxel index containing
      // the reference translation, via voxelIndex() -- see bnb_search.hpp's bnbCellCentre() doc.
      const int64_t trace9_ref_ix = eidos::reloc::voxelIndex(trace9_probe_t.x(), trace9_coarse_lvl.inv_resolution);
      const int64_t trace9_ref_iy = eidos::reloc::voxelIndex(trace9_probe_t.y(), trace9_coarse_lvl.inv_resolution);
      const int64_t trace9_ref_iz = eidos::reloc::voxelIndex(trace9_probe_t.z(), trace9_coarse_lvl.inv_resolution);

      bool trace9_in_kept = false;
      for (const auto & r : kept_roots) {
        if (r.ix == trace9_ref_ix && r.iy == trace9_ref_iy && r.iz == trace9_ref_iz) {
          trace9_in_kept = true;
          break;
        }
      }

      // Reproduce prefilterRoots()'s own scoring pass EXACTLY -- not a re-derivation -- so
      // ref_score/max_score/ties_at_max are the same numbers prefilterRoots() itself computed and
      // handed to std::nth_element(): same strided subsample of size root_prefilter_points_ (see
      // prefilterRoots() above for why strided, not a prefix), same YawDiscretization computed from
      // the FULL rotated query (not the subsample), same coarsest level, and the same hard-coded
      // hit_weight=3 prefilterRoots() itself passes (NOT hit_weight_ -- see its call above; under
      // ScoreMode::DistanceField this argument is unused anyway, but the literal is matched for
      // fidelity regardless of mode).
      const std::size_t trace9_n = prep.rotated_query.size();
      const std::size_t trace9_target =
        std::min(trace9_n, static_cast<std::size_t>(std::max(root_prefilter_points_, 1)));
      std::vector<Eigen::Vector3d> trace9_sub_query;
      trace9_sub_query.reserve(trace9_target);
      if (trace9_target > 0 && trace9_n > 0) {
        const double trace9_step = static_cast<double>(trace9_n) / static_cast<double>(trace9_target);
        for (std::size_t k = 0; k < trace9_target; ++k) {
          std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * trace9_step);
          if (idx >= trace9_n) idx = trace9_n - 1;
          trace9_sub_query.push_back(prep.rotated_query[idx]);
        }
      }

      const eidos::reloc::YawDiscretization trace9_yaw_disc =
        eidos::reloc::YawDiscretization::compute(prep.rotated_query, pyramid_);
      int trace9_ref_score = -1;
      int trace9_max_score = -1;
      int trace9_ties_at_max = 0;
      if (trace9_yaw_disc.max_range > 0.0) {
        const int64_t trace9_n_bins = trace9_yaw_disc.numBins(trace9_coarsest);
        std::vector<int> trace9_scored(roots_.size(), -1);
        // Parallel over roots, exactly like prefilterRoots()'s own loop (and TRACE 4's mirror of
        // it): each iteration writes only its own slot and the pyramid is read-only, so no
        // synchronisation is required.
#pragma omp parallel for schedule(static) num_threads(num_threads_)
        for (std::ptrdiff_t i2 = 0; i2 < static_cast<std::ptrdiff_t>(roots_.size()); ++i2) {
          const std::size_t idx2 = static_cast<std::size_t>(i2);
          const auto & root2 = roots_[idx2];
          const Eigen::Vector3d centre2 =
            eidos::reloc::bnbCellCentre(root2.ix, root2.iy, root2.iz, trace9_coarse_lvl.resolution);
          int best2 = -1;
          for (int64_t k2 = 0; k2 < trace9_n_bins; ++k2) {
            const double yaw2 = trace9_yaw_disc.binCentre(trace9_coarsest, k2);
            const int score2 = eidos::reloc::scorePoseAtLevel(
              pyramid_, trace9_sub_query, centre2, yaw2, trace9_coarsest, 3, 0, nullptr, active_score_mode_);
            if (score2 > best2) best2 = score2;
          }
          trace9_scored[idx2] = best2;
        }

        for (std::size_t idx2 = 0; idx2 < roots_.size(); ++idx2) {
          if (trace9_scored[idx2] > trace9_max_score) trace9_max_score = trace9_scored[idx2];
          const auto & r2 = roots_[idx2];
          if (r2.ix == trace9_ref_ix && r2.iy == trace9_ref_iy && r2.iz == trace9_ref_iz) {
            trace9_ref_score = trace9_scored[idx2];
          }
        }
        for (int s2 : trace9_scored) {
          if (s2 == trace9_max_score) ++trace9_ties_at_max;
        }
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 9 prefilter kept-set: ref_cell=(%ld,%ld,%ld) in_kept=%s kept=%zu ties_at_max=%d "
        "ref_score=%d max_score=%d",
        name_.c_str(),
        static_cast<long>(trace9_ref_ix),  // NOLINT(runtime/int)
        static_cast<long>(trace9_ref_iy),  // NOLINT(runtime/int)
        static_cast<long>(trace9_ref_iz),  // NOLINT(runtime/int)
        trace9_in_kept ? "yes" : "NO",
        kept_roots.size(),
        trace9_ties_at_max,
        trace9_ref_score,
        trace9_max_score);

      // -------------------------------------------------------------------
      // TRACE 9 fine-prefilter: extends the coarse trace above with the FINE, heading-constrained
      // level-0 ranking prefilterRoots() actually uses by default (prefilter_fine_ == true).
      // Calls scoreRootsFine() -- the SAME function prefilterRoots()'s fine path calls, not a
      // re-derivation -- so ref_fine_score/ref_fine_rank/best_fine_* are the same numbers
      // prefilterRoots() itself computed this iteration. `in_kept` reuses trace9_in_kept from the
      // block above rather than re-deriving membership: trace9_in_kept already reflects whichever
      // path (coarse or fine) prefilterRoots() actually took, straight from the `kept_roots` it
      // returned, so this can never disagree with the real outcome.
      // -------------------------------------------------------------------
      if (prefilter_fine_) {
        const auto trace9_fine_scored = scoreRootsFine(trace9_sub_query);

        // Deterministic full ranking, mirroring prefilterRoots()'s own comparator exactly (score
        // descending, idx ascending on ties) so "rank" here means the same thing it means there.
        std::vector<std::size_t> trace9_order(trace9_fine_scored.size());
        for (std::size_t k = 0; k < trace9_order.size(); ++k) trace9_order[k] = k;
        std::sort(trace9_order.begin(), trace9_order.end(), [&](std::size_t a, std::size_t b) {
          const auto & fa = trace9_fine_scored[a];
          const auto & fb = trace9_fine_scored[b];
          return fa.score != fb.score ? fa.score > fb.score : fa.idx < fb.idx;
        });

        int trace9_ref_fine_score = -1;
        std::size_t trace9_ref_fine_rank = 0;  // 1-based; 0 means "reference root not found" (should not happen).
        for (std::size_t rank = 0; rank < trace9_order.size(); ++rank) {
          const auto & fr = trace9_fine_scored[trace9_order[rank]];
          const auto & r2 = roots_[fr.idx];
          if (r2.ix == trace9_ref_ix && r2.iy == trace9_ref_iy && r2.iz == trace9_ref_iz) {
            trace9_ref_fine_score = fr.score;
            trace9_ref_fine_rank = rank + 1;
            break;
          }
        }

        int trace9_best_fine_score = -1;
        Eigen::Vector3d trace9_best_fine_pose = Eigen::Vector3d::Zero();
        double trace9_best_fine_yaw = 0.0;
        int trace9_fine_ties_at_ref = 0;
        for (const auto & fr : trace9_fine_scored) {
          if (fr.score > trace9_best_fine_score) {
            trace9_best_fine_score = fr.score;
            trace9_best_fine_pose = fr.pos;
            trace9_best_fine_yaw = fr.yaw;
          }
          if (fr.score == trace9_ref_fine_score) ++trace9_fine_ties_at_ref;
        }

        RCLCPP_INFO(
          node_->get_logger(),
          "[%s] TRACE 9 fine-prefilter: ref_cell=(%ld,%ld,%ld) in_kept=%s ref_fine_score=%d "
          "ref_fine_rank=%zu/%zu ties_at_ref=%d best_fine_score=%d best_fine_pose=(%.2f,%.2f,%.2f) "
          "yaw=%.1fdeg",
          name_.c_str(),
          static_cast<long>(trace9_ref_ix),   // NOLINT(runtime/int)
          static_cast<long>(trace9_ref_iy),   // NOLINT(runtime/int)
          static_cast<long>(trace9_ref_iz),   // NOLINT(runtime/int)
          trace9_in_kept ? "yes" : "NO",
          trace9_ref_fine_score,
          trace9_ref_fine_rank,
          trace9_fine_scored.size(),
          trace9_fine_ties_at_ref,
          trace9_best_fine_score,
          trace9_best_fine_pose.x(),
          trace9_best_fine_pose.y(),
          trace9_best_fine_pose.z(),
          trace9_best_fine_yaw * 180.0 / M_PI);
      }
    }
  }

  const auto t_prefilter_end = std::chrono::steady_clock::now();
  const double prefilter_ms = std::chrono::duration<double, std::milli>(t_prefilter_end - t_prefilter_start).count();
  if (root_prefilter_keep_ > 0 && root_prefilter_keep_ < static_cast<int>(roots_before)) {
    roots_after = std::min(static_cast<std::size_t>(root_prefilter_keep_), roots_before);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] root prefilter: %zu -> %zu roots, %.1f ms",
    name_.c_str(),
    roots_before,
    roots_after,
    prefilter_ms);

  // Flatten (offset, root-chunk) into one task list so the parallel work below covers roots, not
  // just the (typically <=9-way) roll/pitch offset grid. This is deliberately more, smaller tasks
  // than one per offset -- splitting roots across independent BnB runs slightly weakens pruning,
  // because each chunk maintains its own incumbent rather than sharing a global one, but that
  // trades a little pruning efficiency for much better core utilisation, which measurement showed
  // is the dominant factor on a 992-keyframe map (9-way parallelism on a 16-thread box was leaving
  // most cores idle).
  struct Task
  {
    std::size_t offset_idx;
    std::size_t chunk_idx;
  };
  std::vector<Task> tasks;
  for (std::size_t oi = 0; oi < preps.size(); ++oi) {
    for (std::size_t ci = 0; ci < preps[oi].root_chunks.size(); ++ci) {
      tasks.push_back(Task{oi, ci});
    }
  }

  std::vector<std::vector<ScoredHypothesis>> per_task(tasks.size());
  std::vector<eidos::reloc::SearchStats> per_task_stats(tasks.size());

  const auto t_start = std::chrono::steady_clock::now();

  // Each iteration writes only to its own slot of per_task / per_task_stats (indexed by task id)
  // and reads only const shared state (pyramid_, preps, cfg) -- no shared mutable state, so this
  // is race-free, mirroring the reasoning that already applied when this loop ran over offsets
  // alone. branchAndBound() itself only reads its pyramid/query/roots arguments and writes to its
  // own local frontier/solutions plus the SearchStats reference we pass in, which is unique per
  // task.
#pragma omp parallel for
  for (int t = 0; t < static_cast<int>(tasks.size()); ++t) {
    if (stop_requested_.load()) continue;

    const Task & task = tasks[static_cast<std::size_t>(t)];
    const OffsetPrep & prep = preps[task.offset_idx];
    const std::vector<eidos::reloc::RootCell> & chunk = prep.root_chunks[task.chunk_idx];

    eidos::reloc::SearchConfig scfg;
    scfg.prune_slack = prune_slack_;
    scfg.nms_radius = nms_radius_;
    scfg.hit_weight = hit_weight_;
    // active_score_mode_, not score_mode_: the pyramid may have fallen back to occupancy on
    // max_score_voxels overflow (see buildPyramid()), and the search must score against whatever
    // the pyramid actually built, not what was originally requested.
    scfg.score_mode = active_score_mode_;
    scfg.max_solutions = std::max(num_gicp_candidates_ + 3, 8);
    // Bound the search so it behaves as an anytime algorithm: relocalization runs against a
    // hard `relocalization_timeout`, and an unbounded best-first search over a repetitive map
    // (many hypotheses within `prune_slack` of the best) will not terminate inside it. The
    // budget is split across concurrent tasks so the total stays predictable. Because the
    // frontier is ordered by bound and a greedy dive seeds a real incumbent up front, the
    // best-scoring regions are explored first, so truncation costs optimality, not sanity --
    // and any survivor must still clear the GICP and uniqueness gates.
    scfg.max_nodes = std::max<std::size_t>(
      1000, static_cast<std::size_t>(max_search_nodes_) / std::max<std::size_t>(1, tasks.size()));
    eidos::reloc::SearchStats stats;
    auto hyps = eidos::reloc::branchAndBound(pyramid_, prep.rotated_query, chunk, scfg, stats);
    per_task_stats[static_cast<std::size_t>(t)] = stats;

    std::vector<ScoredHypothesis> local;
    local.reserve(hyps.size());
    for (auto & h : hyps) local.push_back(ScoredHypothesis{h, prep.dr, prep.dp});
    per_task[static_cast<std::size_t>(t)] = std::move(local);
  }

  // Diagnostic: score a known pose (e.g. one reported by another relocalizer) with the exact
  // scorer the search uses. If the probe scores well above the search's best, the scoring and
  // frames are sound and the search or prefilter is at fault; if it scores comparably, the
  // failure is in scoring/frames instead. Cheap, and only runs when explicitly configured.
  //
  // The range-bucket table is the diagnostic that actually separates the two failure modes the
  // outcome alone cannot distinguish: a measured hit rate around 45% at ground truth is anomalous
  // either way, but if near-range points hit at ~90% and far-range at ~20% the cause is map
  // density falling off with query range, whereas if it is flat ~45% across every bucket the cause
  // is a systematic misalignment (frame, TF, or timing) -- and those two need completely different
  // fixes.
  if (debug_probe_pose_.size() >= 4 && !preps.empty()) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
    const double probe_yaw = debug_probe_pose_[3] * M_PI / 180.0;
    const auto & pq = preps[0].rotated_query;
    const double n_pts = static_cast<double>(std::max<std::size_t>(1, pq.size()));

    // -------------------------------------------------------------------
    // TRACE 1-4: pipeline-order diagnostics against the DISCRETE machinery the search actually
    // walks (query -> corridor roots -> yaw bins -> prefilter). Every other diagnostic in this
    // function scores a CONTINUOUS pose; an indexing bug in the discrete machinery (yaw bin <->
    // angle, cell index <-> centre, corridor construction) would be invisible in all of it, so
    // these are checked separately, in pipeline order, once per search.
    // -------------------------------------------------------------------

    // TRACE 1 -- query: point count, the IMU roll/pitch actually applied to de-tilt this scan
    // (the same latched values lidarCallback() reads), and the resulting horizontal-range /
    // body-frame-z envelope. Everything downstream operates on this query, so a broken de-tilt
    // or height band shows up here first.
    {
      double trace_roll = 0.0, trace_pitch = 0.0;
      {
        std::lock_guard<std::mutex> lock(imu_lock_);
        trace_roll = latest_imu_roll_;
        trace_pitch = latest_imu_pitch_;
      }
      double min_range = std::numeric_limits<double>::max();
      double max_range = std::numeric_limits<double>::lowest();
      double min_z = std::numeric_limits<double>::max();
      double max_z = std::numeric_limits<double>::lowest();
      for (const auto & q : query) {
        const double r = std::hypot(q.x(), q.y());
        min_range = std::min(min_range, r);
        max_range = std::max(max_range, r);
        min_z = std::min(min_z, q.z());
        max_z = std::max(max_z, q.z());
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 1 query: n=%zu imu_roll=%.2fdeg imu_pitch=%.2fdeg range=[%.1f,%.1f]m z=[%.2f,%.2f]m",
        name_.c_str(),
        query.size(),
        trace_roll * 180.0 / M_PI,
        trace_pitch * 180.0 / M_PI,
        min_range,
        max_range,
        min_z,
        max_z);
    }

    // TRACE 2 -- corridor roots: does the coarsest-level cell containing the reference
    // translation even exist in roots_? If not, nothing downstream can succeed -- there is no
    // frontier seed anywhere near the true pose, regardless of scoring or search correctness.
    const int trace_coarsest_level = pyramid_.numLevels() - 1;
    const auto & trace_coarse_lvl = pyramid_.level(trace_coarsest_level);
    const int64_t trace_ref_ix = eidos::reloc::voxelIndex(probe_t.x(), trace_coarse_lvl.inv_resolution);
    const int64_t trace_ref_iy = eidos::reloc::voxelIndex(probe_t.y(), trace_coarse_lvl.inv_resolution);
    const int64_t trace_ref_iz = eidos::reloc::voxelIndex(probe_t.z(), trace_coarse_lvl.inv_resolution);
    int trace_root_index = -1;
    {
      double nearest_root_dist = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < roots_.size(); ++i) {
        const auto & r = roots_[i];
        if (r.ix == trace_ref_ix && r.iy == trace_ref_iy && r.iz == trace_ref_iz) {
          trace_root_index = static_cast<int>(i);
        }
        const Eigen::Vector3d centre =
          eidos::reloc::bnbCellCentre(r.ix, r.iy, r.iz, trace_coarse_lvl.resolution);
        nearest_root_dist = std::min(nearest_root_dist, (centre - probe_t).norm());
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 2 corridor roots: ref_cell=(%ld,%ld,%ld) present=%s root_index=%d "
        "total_roots=%zu nearest_root_centre_dist=%.2fm",
        name_.c_str(),
        static_cast<long>(trace_ref_ix),   // NOLINT(runtime/int)
        static_cast<long>(trace_ref_iy),   // NOLINT(runtime/int)
        static_cast<long>(trace_ref_iz),   // NOLINT(runtime/int)
        trace_root_index >= 0 ? "yes" : "NO",
        trace_root_index,
        roots_.size(),
        nearest_root_dist);

      // TRACE 2b -- the reference cell passes the corridor test on paper (its centre is well
      // inside search_corridor horizontally and z_margin vertically of the start pose), so its
      // absence means either no trajectory entry is near enough to generate it, or the generating
      // loop rejects it. Dump both sides so the answer is data, not deduction.
      int near_traj = 0;
      double nearest_traj = std::numeric_limits<double>::max();
      Eigen::Vector3d nearest_traj_pos = Eigen::Vector3d::Zero();
      for (const auto & e : trajectory_) {
        const double d = (e.position - probe_t).norm();
        if (d < 30.0) ++near_traj;
        if (d < nearest_traj) { nearest_traj = d; nearest_traj_pos = e.position; }
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 2b trajectory: %d entries within 30m of ref, nearest at (%.2f,%.2f,%.2f) d=%.2fm "
        "-> its cell=(%ld,%ld,%ld); ref cell=(%ld,%ld,%ld)",
        name_.c_str(), near_traj,
        nearest_traj_pos.x(), nearest_traj_pos.y(), nearest_traj_pos.z(), nearest_traj,
        static_cast<long>(eidos::reloc::voxelIndex(nearest_traj_pos.x(), trace_coarse_lvl.inv_resolution)),
        static_cast<long>(eidos::reloc::voxelIndex(nearest_traj_pos.y(), trace_coarse_lvl.inv_resolution)),
        static_cast<long>(eidos::reloc::voxelIndex(nearest_traj_pos.z(), trace_coarse_lvl.inv_resolution)),
        static_cast<long>(trace_ref_ix), static_cast<long>(trace_ref_iy), static_cast<long>(trace_ref_iz));

      std::string near_roots;
      for (const auto & r : roots_) {
        if (std::llabs(r.ix - trace_ref_ix) <= 1 && std::llabs(r.iy - trace_ref_iy) <= 1) {
          near_roots += "(" + std::to_string(r.ix) + "," + std::to_string(r.iy) + "," + std::to_string(r.iz) + ")";
        }
      }
      RCLCPP_INFO(
        node_->get_logger(), "[%s] TRACE 2c roots adjacent to ref cell: %s",
        name_.c_str(), near_roots.empty() ? "NONE" : near_roots.c_str());
    }

    // TRACE 3 -- yaw binning: bin the reference yaw with the SAME YawDiscretization the search
    // itself computes from this rotated query (YawDiscretization::compute()), then compare the
    // DISCRETE cell/bin score against a CONTINUOUS score of the exact reference pose at the same
    // level. A wild gap between the two means the discretisation -- not the underlying score
    // field -- is where the true pose is being lost.
    const eidos::reloc::YawDiscretization trace_yaw_disc =
      eidos::reloc::YawDiscretization::compute(pq, pyramid_);
    if (trace_yaw_disc.max_range > 0.0) {
      const int64_t n_coarse_bins = trace_yaw_disc.numBins(trace_coarsest_level);
      // Wrap probe_yaw into [0, 2*pi) before binning -- binCentre()/numBins() assume a bin index
      // in [0, n_coarse_bins), and a raw atan2-range yaw can be negative.
      double yaw_wrapped = std::fmod(probe_yaw, 2.0 * M_PI);
      if (yaw_wrapped < 0.0) yaw_wrapped += 2.0 * M_PI;
      int64_t ref_yaw_bin =
        static_cast<int64_t>(yaw_wrapped / (2.0 * M_PI) * static_cast<double>(n_coarse_bins));
      ref_yaw_bin = std::clamp<int64_t>(ref_yaw_bin, 0, n_coarse_bins - 1);
      const double bin_centre = trace_yaw_disc.binCentre(trace_coarsest_level, ref_yaw_bin);
      double yaw_err_deg = (bin_centre - probe_yaw) * 180.0 / M_PI;
      // Wrap the angular error into (-180, 180] for a readable "how far off" figure.
      while (yaw_err_deg > 180.0) yaw_err_deg -= 360.0;
      while (yaw_err_deg <= -180.0) yaw_err_deg += 360.0;

      const Eigen::Vector3d trace_cell_centre = eidos::reloc::bnbCellCentre(
        trace_ref_ix, trace_ref_iy, trace_ref_iz, trace_coarse_lvl.resolution);
      // active_score_mode_/hit_weight_ passed explicitly (not defaulted): scorePoseAtLevel()'s
      // trailing ScoreMode parameter defaults to DistanceField, so omitting it here would
      // silently score against the wrong channel if the pyramid fell back to Occupancy.
      const int discrete_score = eidos::reloc::scorePoseAtLevel(
        pyramid_, pq, trace_cell_centre, bin_centre, trace_coarsest_level, hit_weight_, 0, nullptr,
        active_score_mode_);
      const int continuous_score = eidos::reloc::scorePoseAtLevel(
        pyramid_, pq, probe_t, probe_yaw, trace_coarsest_level, hit_weight_, 0, nullptr, active_score_mode_);
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 3 yaw binning: bin=%ld/%ld bin_centre=%.1fdeg ref_yaw=%.1fdeg err=%.1fdeg "
        "discrete_cell_score=%d continuous_score=%d",
        name_.c_str(),
        static_cast<long>(ref_yaw_bin),    // NOLINT(runtime/int)
        static_cast<long>(n_coarse_bins),  // NOLINT(runtime/int)
        bin_centre * 180.0 / M_PI,
        probe_yaw * 180.0 / M_PI,
        yaw_err_deg,
        discrete_score,
        continuous_score);
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] TRACE 3 yaw binning: degenerate query (max_range<=0), skipped", name_.c_str());
    }

    // TRACE 4 -- prefilter: prefilterRoots() is a HEURISTIC ranking shortcut (see its doc
    // comment) that can legitimately discard the true pose's root before branchAndBound() ever
    // sees it. It does not expose per-root scores, so this reproduces its exact computation --
    // same subsample size, same coarsest-level yaw sweep, same hard-coded hit_weight=3 it itself
    // uses (see prefilterRoots() above) -- for every root, purely to read off where the
    // reference root ranks. Does not call or alter prefilterRoots() itself.
    if (trace_root_index < 0) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] TRACE 4 prefilter: reference cell is not a root (see TRACE 2), skipping",
        name_.c_str());
    } else if (trace_yaw_disc.max_range <= 0.0) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] TRACE 4 prefilter: degenerate query (max_range<=0), skipped", name_.c_str());
    } else {
      const std::size_t trace_n = pq.size();
      const std::size_t trace_target =
        std::min(trace_n, static_cast<std::size_t>(std::max(root_prefilter_points_, 1)));
      std::vector<Eigen::Vector3d> trace_sub_query;
      trace_sub_query.reserve(trace_target);
      if (trace_target > 0 && trace_n > 0) {
        const double step = static_cast<double>(trace_n) / static_cast<double>(trace_target);
        for (std::size_t k = 0; k < trace_target; ++k) {
          std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
          if (idx >= trace_n) idx = trace_n - 1;
          trace_sub_query.push_back(pq[idx]);
        }
      }

      const int64_t trace4_n_bins = trace_yaw_disc.numBins(trace_coarsest_level);
      std::vector<int> trace_root_scores(roots_.size(), -1);
      // Parallel over roots, exactly like prefilterRoots()'s own loop just below: each iteration
      // writes only its own slot and the pyramid is read-only, so no synchronisation is required.
#pragma omp parallel for schedule(static) num_threads(num_threads_)
      for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(roots_.size()); ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);
        const auto & root = roots_[idx];
        const Eigen::Vector3d centre =
          eidos::reloc::bnbCellCentre(root.ix, root.iy, root.iz, trace_coarse_lvl.resolution);
        int best = -1;
        for (int64_t k = 0; k < trace4_n_bins; ++k) {
          const double yaw = trace_yaw_disc.binCentre(trace_coarsest_level, k);
          // Hard-coded hit_weight=3, not hit_weight_: mirrors prefilterRoots()'s own scoring call
          // verbatim (see below), since the point of this trace is to reproduce exactly what
          // that function computes, not to re-score under different weighting.
          const int score = eidos::reloc::scorePoseAtLevel(
            pyramid_, trace_sub_query, centre, yaw, trace_coarsest_level, 3, 0, nullptr, active_score_mode_);
          if (score > best) best = score;
        }
        trace_root_scores[idx] = best;
      }

      const int ref_prefilter_score = trace_root_scores[static_cast<std::size_t>(trace_root_index)];
      int rank = 1;
      for (int s : trace_root_scores) {
        if (s > ref_prefilter_score) ++rank;
      }
      const bool prefilter_active =
        root_prefilter_keep_ > 0 && root_prefilter_keep_ < static_cast<int>(roots_.size());
      const bool survived = !prefilter_active || rank <= root_prefilter_keep_;
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] TRACE 4 prefilter: ref_root_score=%d rank=%d/%zu keep=%d survived=%s",
        name_.c_str(),
        ref_prefilter_score,
        rank,
        roots_.size(),
        root_prefilter_keep_,
        survived ? "yes" : "NO");
    }

    // TRACE 8 -- bound monotonicity walk: TRACE 6 shows the reference pose scores near-perfectly
    // at the coarsest level and TRACE 5 shows the completed search still returns something far
    // worse, with the frontier having emptied on its own (hit_node_cap=0) rather than having been
    // budget-truncated. That combination is only possible if the node containing the reference
    // pose was PRUNED at some level between the coarsest one and the leaf -- and since pruning
    // compares a node's `bound` against `prune_threshold`, a sound bound (which must upper-bound
    // every descendant's exact score, leaf included) can never do that. This walks the reference
    // pose's own path from the coarsest level down to the leaf, at each level computing the
    // EXACT SAME quantity branchAndBound() would compute for that node -- same YawDiscretization,
    // same bnbCellCentre() cell-centre convention, same scorePoseAtLevel() call with the same
    // hit_weight_/active_score_mode_ -- so any drop below the leaf score is not a re-derivation
    // that might itself be wrong, it is the actual number the search's pruning test would have
    // seen.
    //
    // Cell-centre convention used below: bnbCellCentre() in bnb_search.hpp:199-205, i.e.
    // `(index + 0.5) * resolution` -- the CENTRE of the voxel, never the corner. This is exactly
    // what branchAndBound()'s own `boundOf()` lambda uses (bnb_search.hpp:464-469: `bnbCellCentre(
    // ix, iy, iz, resolution)` then `yaw_disc.binCentre(level, yaw_bin)`), and bnbCellCentre()'s
    // own doc comment (bnb_search.hpp:178-198) is the soundness argument the 26-neighbourhood
    // dilation depends on -- so reproducing anything else here (e.g. the cell corner) would not
    // be testing what the search actually does.
    if (trace_yaw_disc.max_range > 0.0) {
      // Yaw-bin lookup at an arbitrary level, generalising TRACE 3's coarsest-level-only version:
      // equal-width partition of [0, 2*pi) into n_bins, reference yaw wrapped into that range
      // first. This is not itself how branchAndBound() descends (it only ever doubles an existing
      // bin index, never re-bins a continuous angle), but by the floor-doubling identity
      // floor(x*2n) in {2*floor(x*n), 2*floor(x*n)+1} it lands on exactly the bin the search's own
      // ck = yaw_bin*2 + dk recursion would have produced, so it is a faithful (if independently
      // computed) stand-in.
      auto yawBinAt = [&](int level) -> int64_t {
        const int64_t n_bins = trace_yaw_disc.numBins(level);
        double yaw_wrapped = std::fmod(probe_yaw, 2.0 * M_PI);
        if (yaw_wrapped < 0.0) yaw_wrapped += 2.0 * M_PI;
        int64_t bin = static_cast<int64_t>(yaw_wrapped / (2.0 * M_PI) * static_cast<double>(n_bins));
        return std::clamp<int64_t>(bin, 0, n_bins - 1);
      };

      // Level-0 (leaf) reference score, via the SAME cell-centre/bin-centre construction used for
      // every other level below. This is the number a sound bound at every coarser level must
      // never fall below -- a parent's bound upper-bounds every descendant's exact score,
      // including the leaf's -- so it is computed once, up front, and reused as the yardstick for
      // every level's `deficit` below (not a hard-coded constant, so this stays correct even if
      // the map/query/config drifts from the numbers in the bug report).
      const auto & trace_leaf_lvl = pyramid_.level(0);
      const int64_t leaf_ix = eidos::reloc::voxelIndex(probe_t.x(), trace_leaf_lvl.inv_resolution);
      const int64_t leaf_iy = eidos::reloc::voxelIndex(probe_t.y(), trace_leaf_lvl.inv_resolution);
      const int64_t leaf_iz = eidos::reloc::voxelIndex(probe_t.z(), trace_leaf_lvl.inv_resolution);
      const Eigen::Vector3d leaf_centre =
        eidos::reloc::bnbCellCentre(leaf_ix, leaf_iy, leaf_iz, trace_leaf_lvl.resolution);
      const double leaf_bin_centre = trace_yaw_disc.binCentre(0, yawBinAt(0));
      const int leaf_ref_score = eidos::reloc::scorePoseAtLevel(
        pyramid_, pq, leaf_centre, leaf_bin_centre, 0, hit_weight_, 0, nullptr, active_score_mode_);

      for (int l = trace_coarsest_level; l >= 0; --l) {
        const auto & lvl_l = pyramid_.level(l);
        const int64_t ix = eidos::reloc::voxelIndex(probe_t.x(), lvl_l.inv_resolution);
        const int64_t iy = eidos::reloc::voxelIndex(probe_t.y(), lvl_l.inv_resolution);
        const int64_t iz = eidos::reloc::voxelIndex(probe_t.z(), lvl_l.inv_resolution);
        const Eigen::Vector3d cell_centre = eidos::reloc::bnbCellCentre(ix, iy, iz, lvl_l.resolution);

        const int64_t n_bins = trace_yaw_disc.numBins(l);
        const int64_t yaw_bin = yawBinAt(l);
        const double bin_centre = trace_yaw_disc.binCentre(l, yaw_bin);
        double yaw_err_deg = (bin_centre - probe_yaw) * 180.0 / M_PI;
        while (yaw_err_deg > 180.0) yaw_err_deg -= 360.0;
        while (yaw_err_deg <= -180.0) yaw_err_deg += 360.0;

        // bound_cc -- THE number branchAndBound() actually computes for the node containing the
        // reference pose at this level: scorePoseAtLevel() at the cell centre / bin centre, via
        // the identical call boundOf() makes (bnb_search.hpp:464-469). Not a re-derivation.
        const int bound_cc = eidos::reloc::scorePoseAtLevel(
          pyramid_, pq, cell_centre, bin_centre, l, hit_weight_, 0, nullptr, active_score_mode_);
        // bound_exact -- the same scorer at the EXACT reference translation/yaw, still evaluated
        // against level l's (possibly dilated/coarse) field. Separates two distinct failure
        // modes: bound_cc << bound_exact means the cell-centre/bin-centre REPRESENTATIVE pose is
        // losing score relative to the true pose (a discretisation/off-centre problem); bound_exact
        // itself dropping means level l's grid is too tight for the exact pose regardless of which
        // representative is used (a dilation-radius/field-construction problem).
        const int bound_exact = eidos::reloc::scorePoseAtLevel(
          pyramid_, pq, probe_t, probe_yaw, l, hit_weight_, 0, nullptr, active_score_mode_);

        // deficit > 0 is a direct, load-bearing proof of an unsound bound at this level: bound_cc
        // is exactly what the search's own pruning test compares against prune_threshold, and a
        // sound bound can never fall below a score (leaf_ref_score) that a descendant of this same
        // node provably achieves.
        const int deficit = leaf_ref_score - bound_cc;
        const bool unsound = deficit > 0;

        RCLCPP_INFO(
          node_->get_logger(),
          "[%s] TRACE 8 bound walk L%d res=%.3fm cell=(%ld,%ld,%ld) centre=(%.2f,%.2f,%.2f) "
          "yawbin=%ld/%ld bin_centre=%.1fdeg yaw_err=%.1fdeg bound_cc=%d bound_exact=%d "
          "leaf_ref=%d deficit=%d UNSOUND=%s",
          name_.c_str(),
          l,
          lvl_l.resolution,
          static_cast<long>(ix),  // NOLINT(runtime/int)
          static_cast<long>(iy),  // NOLINT(runtime/int)
          static_cast<long>(iz),  // NOLINT(runtime/int)
          cell_centre.x(),
          cell_centre.y(),
          cell_centre.z(),
          static_cast<long>(yaw_bin),  // NOLINT(runtime/int)
          static_cast<long>(n_bins),   // NOLINT(runtime/int)
          bin_centre * 180.0 / M_PI,
          yaw_err_deg,
          bound_cc,
          bound_exact,
          leaf_ref_score,
          deficit,
          unsound ? "yes" : "no");
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] TRACE 8 bound walk: degenerate query (max_range<=0), skipped", name_.c_str());
    }

    // Every diagnostic below that goes through scoreBreakdownAtLevel()/scorePoseAtLevel() reports
    // under whichever score mode the pyramid was actually built with (active_score_mode_, which
    // may differ from score_mode_ on a max_score_voxels fallback -- see buildPyramid()), so these
    // numbers are directly comparable to the occupancy figures already measured and recorded in
    // the doc page's Status section. The one deliberate exception is the exact / +/-1 / +/-2 voxel
    // OCCUPANCY tolerance sweep further below: that reference measurement stays as-is regardless of
    // mode, since it is what motivated this work in the first place.
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE active score_mode=%s (requested=%s)", name_.c_str(),
      scoreModeLabel(active_score_mode_), scoreModeLabel(score_mode_));

    const auto probe_bd =
      eidos::reloc::scoreBreakdownAtLevel(pyramid_, pq, probe_t, probe_yaw, 0, hit_weight_, active_score_mode_);
    const double probe_normalized =
      static_cast<double>(probe_bd.raw) / static_cast<double>(std::max(1, probe_bd.max_possible));
    const double probe_hit_fraction = static_cast<double>(probe_bd.hits) / n_pts;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] PROBE pose=(%.1f,%.1f,%.1f) yaw=%.1f -> hits=%d unknown=%d free=%d raw=%d max_possible=%d "
      "normalized=%.3f hit_fraction=%.3f mean_cell_score=%.1f (n=%zu)",
      name_.c_str(), probe_t.x(), probe_t.y(), probe_t.z(), debug_probe_pose_[3], probe_bd.hits,
      probe_bd.unknown, probe_bd.free, probe_bd.raw, probe_bd.max_possible, probe_normalized,
      probe_hit_fraction, probe_bd.mean_cell_score, pq.size());

    // Rebuild the query WITHOUT the min_height/max_height band, from the same buffered scan the
    // real query is derived from, so the bucket table below can be recomputed to show whether the
    // band -- rather than the search or the map itself -- is what is throwing away the matching
    // structure. This can only isolate the QUERY side of the band (the map/pyramid was rasterized
    // with the band applied and cannot be un-filtered without a full rebuild), but a query point
    // that hits with the band off and was simply never tested with it on is still a direct,
    // reliable signal that the band is costing structure.
    std::vector<Eigen::Vector3d> unfiltered_query;
    {
      small_gicp::PointCloud::Ptr scan_copy;
      {
        std::lock_guard<std::mutex> lock(scan_lock_);
        scan_copy = latest_scan_;
      }
      double roll = 0.0, pitch = 0.0;
      {
        std::lock_guard<std::mutex> lock(imu_lock_);
        roll = latest_imu_roll_;
        pitch = latest_imu_pitch_;
      }
      if (scan_copy) {
        const Eigen::Matrix3d r_detilt = rotYX(pitch, roll);
        const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
        unfiltered_query.reserve(scan_copy->size());
        for (std::size_t i = 0; i < scan_copy->size(); ++i) {
          const Eigen::Vector3d p = r_detilt * scan_copy->point(i).head<3>();
          if (max_range_sq > 0.0 && p.head<2>().squaredNorm() > max_range_sq) continue;
          unfiltered_query.push_back(p);
        }
      }
    }

    // Range-bucket hit-fraction table: 0-10 / 10-20 / 20-30 / 30-40 m, at the probe pose. Run once
    // against the real (height-band-filtered) query and once against `unfiltered_query`.
    constexpr double kBucketEdges[5] = {0.0, 10.0, 20.0, 30.0, 40.0};
    auto rangeBucketRow = [&](const std::vector<Eigen::Vector3d> & q) {
      std::string row;
      for (int b = 0; b < 4; ++b) {
        std::vector<Eigen::Vector3d> sub;
        for (const auto & p : q) {
          const double r = std::hypot(p.x(), p.y());
          if (r >= kBucketEdges[b] && r < kBucketEdges[b + 1]) sub.push_back(p);
        }
        if (sub.empty()) {
          row += " [" + std::to_string(static_cast<int>(kBucketEdges[b])) + "-" +
            std::to_string(static_cast<int>(kBucketEdges[b + 1])) + "m n=0]";
          continue;
        }
        const auto bd =
          eidos::reloc::scoreBreakdownAtLevel(pyramid_, sub, probe_t, probe_yaw, 0, hit_weight_, active_score_mode_);
        const double hit_frac = static_cast<double>(bd.hits) / static_cast<double>(sub.size());
        row += " [" + std::to_string(static_cast<int>(kBucketEdges[b])) + "-" +
          std::to_string(static_cast<int>(kBucketEdges[b + 1])) + "m n=" + std::to_string(sub.size()) +
          " hit=" + std::to_string(static_cast<int>(hit_frac * 100.0 + 0.5)) + "%]";
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE range-bucket hit%% (height band ON): %s", name_.c_str(),
      rangeBucketRow(pq).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE range-bucket hit%% (height band OFF):%s", name_.c_str(),
      rangeBucketRow(unfiltered_query).c_str());

    // TOLERANCE SWEEP: the decisive diagnostic for telling the two candidate causes of the ~40%
    // (~52% at 0-10m) hit fraction measured at ground truth apart. exact == level(0).hit(), the
    // same test the search leaf uses; +/-1 voxel == level(0).hitBound() (the 26-neighbourhood
    // probe already used for the coarse-level bound); +/-2 voxels is a hand-rolled 5x5x5 probe
    // over the same level-0 exact set (level 0 is never dilated, so this is a direct, honest
    // widening of the test, not a reuse of a coarser level's inflated occupancy). A jump from
    // ~40% at exact to ~85%+ at +/-1 voxel would mean the query is simply offset from the map by
    // about a voxel -- pose, extrinsic, or quantisation error -- and relaxing the test recovers
    // it (systematic misalignment). Staying near the ~40-52% measured at exact even at +/-2
    // voxels would mean those query points are genuinely not represented in the map at this
    // location (vegetation, dynamic objects, seasonal change), which no amount of tolerance can
    // fix. Range-bucketed the same way as the table above so a range-dependent split within a
    // single tolerance level is not missed either.
    const auto & probe_lvl0 = pyramid_.level(0);
    auto probeTransform = [&](const Eigen::Vector3d & q) {
      const double c = std::cos(probe_yaw), s = std::sin(probe_yaw);
      return Eigen::Vector3d(
        c * q.x() - s * q.y() + probe_t.x(), s * q.x() + c * q.y() + probe_t.y(), q.z() + probe_t.z());
    };
    auto hitExact = [&](const Eigen::Vector3d & p) { return probe_lvl0.hit(p); };
    auto hitPm1 = [&](const Eigen::Vector3d & p) { return probe_lvl0.hitBound(p); };
    auto hitPm2 = [&](const Eigen::Vector3d & p) {
      const int64_t vx = eidos::reloc::voxelIndex(p.x(), probe_lvl0.inv_resolution);
      const int64_t vy = eidos::reloc::voxelIndex(p.y(), probe_lvl0.inv_resolution);
      const int64_t vz = eidos::reloc::voxelIndex(p.z(), probe_lvl0.inv_resolution);
      for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
          for (int dz = -2; dz <= 2; ++dz) {
            if (probe_lvl0.voxels.find(eidos::reloc::packVoxel(vx + dx, vy + dy, vz + dz)) !=
                probe_lvl0.voxels.end()) {
              return true;
            }
          }
        }
      }
      return false;
    };
    auto toleranceRow = [&](auto && test) {
      std::size_t total_hits = 0;
      std::size_t bucket_hits[4] = {0, 0, 0, 0};
      std::size_t bucket_n[4] = {0, 0, 0, 0};
      for (const auto & q : pq) {
        const bool hit = test(probeTransform(q));
        if (hit) ++total_hits;
        const double r = std::hypot(q.x(), q.y());
        for (int b = 0; b < 4; ++b) {
          if (r >= kBucketEdges[b] && r < kBucketEdges[b + 1]) {
            ++bucket_n[b];
            if (hit) ++bucket_hits[b];
            break;
          }
        }
      }
      std::string row =
        std::to_string(static_cast<int>(static_cast<double>(total_hits) / n_pts * 100.0 + 0.5)) + "% overall";
      for (int b = 0; b < 4; ++b) {
        const double frac =
          bucket_n[b] > 0 ? static_cast<double>(bucket_hits[b]) / static_cast<double>(bucket_n[b]) : 0.0;
        row += " [" + std::to_string(static_cast<int>(kBucketEdges[b])) + "-" +
          std::to_string(static_cast<int>(kBucketEdges[b + 1])) + "m n=" + std::to_string(bucket_n[b]) +
          " hit=" + std::to_string(static_cast<int>(frac * 100.0 + 0.5)) + "%]";
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE tolerance sweep exact (0 vox):    %s", name_.c_str(),
      toleranceRow(hitExact).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE tolerance sweep +/-1 voxel:       %s", name_.c_str(),
      toleranceRow(hitPm1).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE tolerance sweep +/-2 voxels:      %s", name_.c_str(),
      toleranceRow(hitPm2).c_str());

    // Sweep yaw at the probe's translation. If the score is flat in yaw, the query is
    // rotation-degenerate (dominated by ground returns) and no yaw search can disambiguate it.
    // Logs normalized% (the ternary score the gates actually use) next to hit_fraction% (the raw
    // occupancy hit rate) so the two are never confused in a field log.
    std::string yaw_row;
    for (int deg = 0; deg < 360; deg += 30) {
      const auto bd = eidos::reloc::scoreBreakdownAtLevel(
        pyramid_, pq, probe_t, deg * M_PI / 180.0, 0, hit_weight_, active_score_mode_);
      // raw / max_possible, not raw / (hit_weight * n): under distance_field max_possible is
      // 255 * n rather than hit_weight * n, so this is the one formula that is correct in both
      // modes (see ScoreBreakdown::max_possible in bnb_search.hpp).
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      yaw_row += " " + std::to_string(deg) + ":" + std::to_string(static_cast<int>(norm * 100.0 + 0.5)) + "/" +
        std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
    }
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE yaw sweep (normalized%%/hit_fraction%%):%s", name_.c_str(),
      yaw_row.c_str());

    // FINE LOCAL SWEEPS: the yaw sweep just above steps 30 degrees, coarse enough that it cannot
    // tell a sharp local peak at the true pose (good: the search has something to converge onto)
    // from a broad, nearly-flat lobe (bad: poor local observability at this location) apart. 1
    // degree steps over +/-10 degrees, and 0.5 m steps over +/-3 m in x and y (the other two axes
    // held at their probe values), resolve that -- if hit_fraction is essentially flat across
    // these narrow windows, the true pose is not even a local maximum of the score and the search
    // has no gradient to find it by.
    std::string fine_yaw_row;
    for (int ddeg = -10; ddeg <= 10; ++ddeg) {
      const double yaw = probe_yaw + static_cast<double>(ddeg) * M_PI / 180.0;
      const auto bd =
        eidos::reloc::scoreBreakdownAtLevel(pyramid_, pq, probe_t, yaw, 0, hit_weight_, active_score_mode_);
      // raw / max_possible, not raw / (hit_weight * n): under distance_field max_possible is
      // 255 * n rather than hit_weight * n, so this is the one formula that is correct in both
      // modes (see ScoreBreakdown::max_possible in bnb_search.hpp).
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      fine_yaw_row += " " + std::to_string(ddeg) + ":" + std::to_string(static_cast<int>(norm * 100.0 + 0.5)) +
        "/" + std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
    }
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE fine yaw sweep +/-10deg@1deg (normalized%%/hit_fraction%%):%s",
      name_.c_str(), fine_yaw_row.c_str());

    // offsetLabel renders tenths-of-a-metre steps (e.g. i=-25 -> "-2.5") without pulling in
    // <cstdio>/<sstream>, matching the plain std::to_string style already used in this block.
    auto offsetLabel = [](int tenths) {
      const int mag = std::abs(tenths);
      return std::string(tenths < 0 ? "-" : "") + std::to_string(mag / 10) + "." + std::to_string(mag % 10);
    };
    std::string x_row;
    for (int t10 = -30; t10 <= 30; t10 += 5) {
      const Eigen::Vector3d t(probe_t.x() + static_cast<double>(t10) / 10.0, probe_t.y(), probe_t.z());
      const auto bd =
        eidos::reloc::scoreBreakdownAtLevel(pyramid_, pq, t, probe_yaw, 0, hit_weight_, active_score_mode_);
      // raw / max_possible, not raw / (hit_weight * n): under distance_field max_possible is
      // 255 * n rather than hit_weight * n, so this is the one formula that is correct in both
      // modes (see ScoreBreakdown::max_possible in bnb_search.hpp).
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      x_row += " " + offsetLabel(t10) + ":" + std::to_string(static_cast<int>(norm * 100.0 + 0.5)) + "/" +
        std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
    }
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE x sweep +/-3m@0.5m (normalized%%/hit_fraction%%):%s", name_.c_str(),
      x_row.c_str());

    std::string y_row;
    for (int t10 = -30; t10 <= 30; t10 += 5) {
      const Eigen::Vector3d t(probe_t.x(), probe_t.y() + static_cast<double>(t10) / 10.0, probe_t.z());
      const auto bd =
        eidos::reloc::scoreBreakdownAtLevel(pyramid_, pq, t, probe_yaw, 0, hit_weight_, active_score_mode_);
      // raw / max_possible, not raw / (hit_weight * n): under distance_field max_possible is
      // 255 * n rather than hit_weight * n, so this is the one formula that is correct in both
      // modes (see ScoreBreakdown::max_possible in bnb_search.hpp).
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      y_row += " " + offsetLabel(t10) + ":" + std::to_string(static_cast<int>(norm * 100.0 + 0.5)) + "/" +
        std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
    }
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE y sweep +/-3m@0.5m (normalized%%/hit_fraction%%):%s", name_.c_str(),
      y_row.c_str());

    // CHANCE BASELINE: how far above chance the true yaw's hit fraction sits at the probe
    // TRANSLATION. This is the statistic that matters if the raw hit count measured at ground
    // truth turns out to be driven mostly by how dense the map happens to be right there rather
    // than by the query actually being aligned to it: if many headings at this same spot score
    // similarly, true/mean sits near 1.0 and the hit count is not telling us much about
    // alignment; a genuinely well-aligned pose should sit well above the mean of the 36 headings
    // sampled here.
    double chance_hit_sum = 0.0;
    double chance_hit_max = 0.0;
    int chance_samples = 0;
    // Also track the ACTIVE-mode normalized score across the same 36 yaws, and the true yaw's
    // RANK among them (1 = best). This is the statistic that actually decides whether this change
    // worked: under occupancy the true yaw was measured to rank only 5th of 36 (see the doc page's
    // Status section) -- no peak at the true pose. If the distance field is discriminative, this
    // rank should move decisively toward 1/36, directly comparable to that occupancy figure.
    double chance_norm_sum = 0.0;
    double chance_norm_max = 0.0;
    int true_yaw_rank = 1;
    for (int deg = 0; deg < 360; deg += 10) {
      const auto bd = eidos::reloc::scoreBreakdownAtLevel(
        pyramid_, pq, probe_t, static_cast<double>(deg) * M_PI / 180.0, 0, hit_weight_, active_score_mode_);
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      chance_hit_sum += hitf;
      chance_hit_max = std::max(chance_hit_max, hitf);
      chance_norm_sum += norm;
      chance_norm_max = std::max(chance_norm_max, norm);
      if (norm > probe_normalized) ++true_yaw_rank;
      ++chance_samples;
    }
    const double chance_hit_mean = chance_samples > 0 ? chance_hit_sum / static_cast<double>(chance_samples) : 0.0;
    const double chance_ratio = chance_hit_mean > 0.0 ? probe_hit_fraction / chance_hit_mean : 0.0;
    const double chance_norm_mean = chance_samples > 0 ? chance_norm_sum / static_cast<double>(chance_samples) : 0.0;
    const double chance_norm_ratio = chance_norm_mean > 0.0 ? probe_normalized / chance_norm_mean : 0.0;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] PROBE chance baseline (36 yaws @10deg at probe translation): mean_hit=%.3f max_hit=%.3f "
      "true_hit=%.3f true/mean=%.3f",
      name_.c_str(), chance_hit_mean, chance_hit_max, probe_hit_fraction, chance_ratio);
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] PROBE chance baseline active score_mode=%s (36 yaws @10deg): mean_norm=%.3f "
      "max_norm=%.3f true_norm=%.3f true/mean=%.3f true_yaw_rank=%d/%d",
      name_.c_str(), scoreModeLabel(active_score_mode_), chance_norm_mean, chance_norm_max,
      probe_normalized, chance_norm_ratio, true_yaw_rank, chance_samples);

    // Sweep z at the probe's translation and yaw, to expose any vertical frame offset.
    std::string z_row;
    for (double dz = -4.0; dz <= 4.01; dz += 1.0) {
      const Eigen::Vector3d t(probe_t.x(), probe_t.y(), probe_t.z() + dz);
      const auto bd =
        eidos::reloc::scoreBreakdownAtLevel(pyramid_, pq, t, probe_yaw, 0, hit_weight_, active_score_mode_);
      // raw / max_possible, not raw / (hit_weight * n): under distance_field max_possible is
      // 255 * n rather than hit_weight * n, so this is the one formula that is correct in both
      // modes (see ScoreBreakdown::max_possible in bnb_search.hpp).
      const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      const double hitf = static_cast<double>(bd.hits) / n_pts;
      z_row += " " + std::to_string(static_cast<int>(dz)) + ":" + std::to_string(static_cast<int>(norm * 100.0 + 0.5)) +
        "/" + std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
    }
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE dz sweep (normalized%%/hit_fraction%%):%s", name_.c_str(), z_row.c_str());

    // ROLL/PITCH RESIDUAL SWEEP. The 4-DOF search cannot correct roll or pitch -- it fixes them
    // from the IMU-derived de-tilt and searches only (x, y, z, yaw) -- so a residual attitude
    // error is invisible to every other diagnostic here and uncorrectable by the search, yet it
    // displaces a point at 40 m by 0.7 m per degree, which is most of a voxel at the default
    // resolution.
    //
    // The signature that motivated this: on-route, the live scan hits 85% at +/-1 voxel in the
    // 0-10 m bucket but only 29% at 20-30 m. A translation error is range-independent and a yaw
    // error was independently excluded (GICP from truth reports 0.6 deg of yaw correction), so a
    // range-dependent falloff of that shape is an attitude error about a horizontal axis.
    //
    // Sweeps the de-tilt applied to the QUERY rather than moving the pose, since that is exactly
    // the degree of freedom `rp_search_range`/`rp_search_steps` would search: a non-zero argmax
    // here means the shipped `rp_search_steps: 1` (trust the IMU) is the wrong default for this
    // platform, and says how wide the search would have to be.
    {
      std::string best_line;
      double best_norm = -1.0;
      double best_dr = 0.0, best_dp = 0.0;
      std::string pitch_row;
      for (int ip = -8; ip <= 8; ++ip) {
        const double dp = static_cast<double>(ip) * 0.5 * M_PI / 180.0;
        double row_best = -1.0;
        for (int ir = -8; ir <= 8; ++ir) {
          const double dr = static_cast<double>(ir) * 0.5 * M_PI / 180.0;
          const Eigen::Matrix3d radj = rotYX(dp, dr);
          std::vector<Eigen::Vector3d> rq;
          rq.reserve(pq.size());
          for (const auto & p : pq) rq.push_back(radj * p);
          const auto bd = eidos::reloc::scoreBreakdownAtLevel(
            pyramid_, rq, probe_t, probe_yaw, 0, hit_weight_, active_score_mode_);
          const double norm = static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
          if (norm > row_best) row_best = norm;
          if (norm > best_norm) {
            best_norm = norm;
            best_dr = static_cast<double>(ir) * 0.5;
            best_dp = static_cast<double>(ip) * 0.5;
          }
        }
        pitch_row += " " + std::to_string(static_cast<int>(static_cast<double>(ip) * 0.5 * 10)) + "d:" +
          std::to_string(static_cast<int>(row_best * 100.0 + 0.5));
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] PROBE roll/pitch sweep +/-4deg@0.5deg: argmax roll=%+.1fdeg pitch=%+.1fdeg norm=%.3f "
        "(at zero offset norm=%.3f)",
        name_.c_str(), best_dr, best_dp, best_norm, probe_normalized);
      RCLCPP_INFO(
        node_->get_logger(), "[%s] PROBE roll/pitch sweep best-per-pitch (pitch_deg*10:norm%%):%s",
        name_.c_str(), pitch_row.c_str());
    }

    // EXACT vs +/-1 VOXEL DISCRIMINATION CHECK: the tolerance sweep above already showed the
    // OVERALL hit fraction jumping from ~38% (exact) toward the 82%+ measured at +/-1 voxel, but
    // that alone cannot tell "tolerance restores alignment" apart from "tolerance just raises
    // every pose's score by about the same amount, exact fails to discriminate them (a 60deg
    // wrong yaw scored 48% against the true yaw's 38% at exact), so the two existing diagnostics
    // that would show discrimination -- the coarse chance-baseline yaw sweep and the fine local
    // sweeps -- are recomputed here under hitPm1 next to their existing hitExact numbers. Reuses
    // probeTransform's rotation math but lets t/yaw vary per sample, which the fixed-probe-pose
    // lambdas above cannot.
    auto hitFractionAt = [&](const Eigen::Vector3d & t, double yaw, auto && test) {
      const double c = std::cos(yaw), s = std::sin(yaw);
      std::size_t hits = 0;
      for (const auto & q : pq) {
        const Eigen::Vector3d p(
          c * q.x() - s * q.y() + t.x(), s * q.x() + c * q.y() + t.y(), q.z() + t.z());
        if (test(p)) ++hits;
      }
      return static_cast<double>(hits) / n_pts;
    };

    // 1. COARSE YAW SWEEP: 36 yaws at 10 degree steps at the probe translation, hit fraction only,
    // under both tests. Same grid the exact-only chance baseline further above already samples.
    constexpr int kNumCoarseYaws = 36;
    std::vector<double> coarse_exact(kNumCoarseYaws), coarse_pm1(kNumCoarseYaws);
    for (int i = 0; i < kNumCoarseYaws; ++i) {
      const double yaw = static_cast<double>(i * 10) * M_PI / 180.0;
      coarse_exact[static_cast<std::size_t>(i)] = hitFractionAt(probe_t, yaw, hitExact);
      coarse_pm1[static_cast<std::size_t>(i)] = hitFractionAt(probe_t, yaw, hitPm1);
    }
    const double true_hit_exact = hitFractionAt(probe_t, probe_yaw, hitExact);
    const double true_hit_pm1 = hitFractionAt(probe_t, probe_yaw, hitPm1);
    auto formatCoarseRow = [&](const std::vector<double> & vals) {
      std::string row;
      for (int i = 0; i < kNumCoarseYaws; ++i) {
        row += " " + std::to_string(i * 10) + ":" +
          std::to_string(static_cast<int>(vals[static_cast<std::size_t>(i)] * 100.0 + 0.5));
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE coarse yaw sweep 36@10deg hit_fraction%% exact:   %s", name_.c_str(),
      formatCoarseRow(coarse_exact).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE coarse yaw sweep 36@10deg hit_fraction%% +/-1vox: %s", name_.c_str(),
      formatCoarseRow(coarse_pm1).c_str());

    // 2. CHANCE BASELINE under each test, plus the RANK of the true yaw among the 36 (1 = best).
    // Rank is the number that actually settles the question: exact currently ranks the true yaw
    // poorly (a wrong yaw outscored it, per the finding above) because exact-containment is
    // measuring local map density, not alignment; if tolerance is the right fix, true_rank under
    // +/-1vox should climb sharply toward 1, not just true_hit/mean rising in lockstep with every
    // other yaw's hit fraction.
    double exact_sum = 0.0, exact_max = 0.0, pm1_sum = 0.0, pm1_max = 0.0;
    int exact_rank = 1, pm1_rank = 1;
    for (int i = 0; i < kNumCoarseYaws; ++i) {
      const double ev = coarse_exact[static_cast<std::size_t>(i)];
      const double pv = coarse_pm1[static_cast<std::size_t>(i)];
      exact_sum += ev;
      pm1_sum += pv;
      exact_max = std::max(exact_max, ev);
      pm1_max = std::max(pm1_max, pv);
      if (ev > true_hit_exact) ++exact_rank;
      if (pv > true_hit_pm1) ++pm1_rank;
    }
    const double exact_mean = exact_sum / static_cast<double>(kNumCoarseYaws);
    const double pm1_mean = pm1_sum / static_cast<double>(kNumCoarseYaws);
    const double exact_chance_ratio = exact_mean > 0.0 ? true_hit_exact / exact_mean : 0.0;
    const double pm1_chance_ratio = pm1_mean > 0.0 ? true_hit_pm1 / pm1_mean : 0.0;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] PROBE chance baseline exact:   mean_hit=%.3f max_hit=%.3f true_hit=%.3f true/mean=%.3f "
      "true_rank=%d/%d",
      name_.c_str(), exact_mean, exact_max, true_hit_exact, exact_chance_ratio, exact_rank, kNumCoarseYaws);
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] PROBE chance baseline +/-1vox: mean_hit=%.3f max_hit=%.3f true_hit=%.3f true/mean=%.3f "
      "true_rank=%d/%d",
      name_.c_str(), pm1_mean, pm1_max, true_hit_pm1, pm1_chance_ratio, pm1_rank, kNumCoarseYaws);

    // 3. Same exact vs +/-1vox comparison over the fine local windows already swept above (fine
    // yaw, x, y), to see whether a LOCAL peak at the true pose appears under tolerance where exact
    // has none (exact was measured flat at 35-41% over +/-10deg above). Hit fractions only, not
    // normalized%, to keep the rows readable.
    auto formatFineYawRow = [&](auto && test) {
      std::string row;
      for (int ddeg = -10; ddeg <= 10; ++ddeg) {
        const double yaw = probe_yaw + static_cast<double>(ddeg) * M_PI / 180.0;
        const double hitf = hitFractionAt(probe_t, yaw, test);
        row += " " + std::to_string(ddeg) + ":" + std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE fine yaw sweep +/-10deg@1deg hit_fraction%% exact:   %s", name_.c_str(),
      formatFineYawRow(hitExact).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE fine yaw sweep +/-10deg@1deg hit_fraction%% +/-1vox: %s", name_.c_str(),
      formatFineYawRow(hitPm1).c_str());

    auto formatXRow = [&](auto && test) {
      std::string row;
      for (int t10 = -30; t10 <= 30; t10 += 5) {
        const Eigen::Vector3d t(probe_t.x() + static_cast<double>(t10) / 10.0, probe_t.y(), probe_t.z());
        const double hitf = hitFractionAt(t, probe_yaw, test);
        row += " " + offsetLabel(t10) + ":" + std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE x sweep +/-3m@0.5m hit_fraction%% exact:   %s", name_.c_str(),
      formatXRow(hitExact).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE x sweep +/-3m@0.5m hit_fraction%% +/-1vox: %s", name_.c_str(),
      formatXRow(hitPm1).c_str());

    auto formatYRow = [&](auto && test) {
      std::string row;
      for (int t10 = -30; t10 <= 30; t10 += 5) {
        const Eigen::Vector3d t(probe_t.x(), probe_t.y() + static_cast<double>(t10) / 10.0, probe_t.z());
        const double hitf = hitFractionAt(t, probe_yaw, test);
        row += " " + offsetLabel(t10) + ":" + std::to_string(static_cast<int>(hitf * 100.0 + 0.5));
      }
      return row;
    };
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE y sweep +/-3m@0.5m hit_fraction%% exact:   %s", name_.c_str(),
      formatYRow(hitExact).c_str());
    RCLCPP_INFO(
      node_->get_logger(), "[%s] PROBE y sweep +/-3m@0.5m hit_fraction%% +/-1vox: %s", name_.c_str(),
      formatYRow(hitPm1).c_str());
  }

  // ===========================================================================================
  // TEMPORARY DIAGNOSTIC: height-band discrimination sweep.
  //
  // Two independent scoring functions (ternary occupancy and the distance field) both failed to
  // peak at the verified ground-truth pose on ring_road.map, and a randomly rotated scan already
  // scores ~76% of the true pose's value. That is the signature of a query whose points are
  // nearly pose-invariant with respect to the map. This sweep asks whether the body-frame height
  // band (min_height_/max_height_, applied relative to base_link, which here is base_footprint --
  // GROUND level, not the LiDAR mount) is selecting a non-discriminative slice of the scene:
  // Part 1 dumps z histograms of the raw scan and nearby prior-map keyframes (no band applied) so
  // the vertical distribution of available structure is visible directly, in metres above the
  // road surface. Part 2 re-scores several candidate bands (including today's default) against a
  // LOCAL pyramid built only from points in that band: "canopy" is the negative control (foliage
  // is space-filling and seasonally unstable, so it should discriminate WORST), "struct" isolates
  // the stable ground-level structure (curbs, barriers, vehicles, walls, pole bases), and
  // "off"/"withground" test whether the road surface -- uninformative in x/y/yaw but present
  // everywhere -- is diluting the score. A band whose yaw_rank lands at 1/36 with a clearly >1
  // yaw_ratio is the one the scorer should actually be using. This is NOT wired into the live
  // search or scoring path in any way -- it is read-only diagnostics, gated behind
  // debug_band_sweep_ (default false) and debug_probe_pose_, and is meant to be deleted once the
  // question above is answered.
  // ===========================================================================================
  if (debug_band_sweep_ && debug_probe_pose_.size() >= 4) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
    const double probe_yaw = debug_probe_pose_[3] * M_PI / 180.0;

    // -----------------------------------------------------------------------------------------
    // Part 1 setup: buffered live scan, de-tilted and range-filtered exactly like the real query
    // builder (lidarCallback()), but WITHOUT the height-band filter -- the whole point of this
    // diagnostic is to see what the band throws away, so the query it starts from must not
    // already have thrown it away.
    // -----------------------------------------------------------------------------------------
    small_gicp::PointCloud::Ptr scan_copy;
    {
      std::lock_guard<std::mutex> lock(scan_lock_);
      scan_copy = latest_scan_;
    }
    double roll = 0.0, pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(imu_lock_);
      roll = latest_imu_roll_;
      pitch = latest_imu_pitch_;
    }
    const Eigen::Matrix3d r_detilt = rotYX(pitch, roll);
    const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
    std::vector<Eigen::Vector3d> scan_body;
    if (scan_copy) {
      scan_body.reserve(scan_copy->size());
      for (std::size_t i = 0; i < scan_copy->size(); ++i) {
        const Eigen::Vector3d p = r_detilt * scan_copy->point(i).head<3>();
        if (max_range_sq > 0.0 && p.head<2>().squaredNorm() > max_range_sq) continue;
        scan_body.push_back(p);
      }
    }

    // Prior-map keyframes within 100 m horizontally of the probe. Built once here and reused by
    // Part 2 below (the yaw/xy sweep needs the SAME set of nearby keyframes the histogram used),
    // so the getKeyList()/getCloudIndex() walk over the whole map only happens once per search,
    // not once per candidate band.
    std::vector<std::pair<gtsam::Key, Eigen::Isometry3d>> near_kfs;
    {
      auto key_list = map_manager_->getKeyList();
      for (gtsam::Key key : key_list) {
        if (!map_manager_->isPriorMapKey(key)) continue;
        int idx = map_manager_->getCloudIndex(key);
        if (idx < 0 || static_cast<std::size_t>(idx) >= poses6d_->points.size()) continue;
        Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(idx)]);
        Eigen::Isometry3d T;
        T.matrix() = world_t.matrix().cast<double>();
        if ((T.translation().head<2>() - probe_t.head<2>()).norm() > 100.0) continue;
        near_kfs.emplace_back(key, T);
      }
    }

    // Same two-branch retrieval (PCL-typed, then small_gicp-typed) and source-key selection
    // insertKeyframeCloud()/buildPyramid() use, factored out so it can be reused across every
    // keyframe and every band below. Returns BODY-frame points -- i.e. NOT transformed by a
    // keyframe's world pose, and with no height band applied -- since that is exactly what both
    // the histogram and the per-band local pyramid need before they apply their own filter.
    const std::string cloud_suffix = "/cloud";
    const bool has_fallback = pointcloud_from_.size() >= cloud_suffix.size() &&
      pointcloud_from_.compare(pointcloud_from_.size() - cloud_suffix.size(), cloud_suffix.size(), cloud_suffix) ==
        0;
    const std::string fallback_key = has_fallback
      ? pointcloud_from_.substr(0, pointcloud_from_.size() - cloud_suffix.size()) + "/gicp_cloud"
      : std::string();
    auto retrieveBody = [&](gtsam::Key key, std::vector<Eigen::Vector3d> & out) -> bool {
      auto tryKey = [&](const std::string & data_key) -> bool {
        auto pcl_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(key, data_key);
        if (pcl_opt.has_value() && *pcl_opt && !(*pcl_opt)->empty()) {
          out.reserve(out.size() + (*pcl_opt)->points.size());
          for (const auto & pt : (*pcl_opt)->points) out.emplace_back(pt.x, pt.y, pt.z);
          return true;
        }
        auto gicp_opt = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(key, data_key);
        if (gicp_opt.has_value() && *gicp_opt && !(*gicp_opt)->empty()) {
          out.reserve(out.size() + (*gicp_opt)->size());
          for (std::size_t i = 0; i < (*gicp_opt)->size(); ++i) {
            const Eigen::Vector3d p = (*gicp_opt)->point(i).head<3>();
            out.push_back(p);
          }
          return true;
        }
        return false;
      };
      if (has_fallback && prefer_downsampled_source_) {
        if (tryKey(fallback_key)) return true;
        return tryKey(pointcloud_from_);
      }
      if (tryKey(pointcloud_from_)) return true;
      return has_fallback && tryKey(fallback_key);
    };

    // Retrieve every near keyframe's body-frame points ONCE and cache them: Part 2 below needs
    // ALL of near_kfs for EVERY candidate band, and retrieving the same clouds from map_manager_
    // repeatedly (6 bands x however many keyframes) would be both slow and pointless, since
    // retrieveBody()'s result does not depend on the band -- only the height filter applied to it
    // does. The histogram just below uses a prefix of this same cache.
    std::vector<std::vector<Eigen::Vector3d>> near_kf_bodies(near_kfs.size());
    for (std::size_t i = 0; i < near_kfs.size(); ++i) {
      retrieveBody(near_kfs[i].first, near_kf_bodies[i]);
    }

    // offsetLabel-style tenths-of-a-metre formatter for histogram bin labels (bins are 0.5 m
    // wide), matching the plain std::to_string style already used by the PROBE block above.
    auto formatHalf = [](double v) {
      const int tenths = static_cast<int>(std::lround(v * 10.0));
      const int mag = std::abs(tenths);
      return std::string(tenths < 0 ? "-" : "") + std::to_string(mag / 10) + "." + std::to_string(mag % 10);
    };

    // Z HISTOGRAM: bins body-frame z into 0.5 m buckets over [-3.0, 8.0) m (22 bins) and reports
    // the percentage of points landing in each non-empty bin, plus the 5th/50th/95th percentile
    // z. This is what makes "the band is above (or below) the discriminative structure" directly
    // checkable, rather than inferred from downstream score numbers. Split across two log lines
    // (first half / second half of the bin range) to stay well under the ~900 char budget even
    // when every bin is populated.
    constexpr double kHistLo = -3.0;
    constexpr double kHistHi = 8.0;
    constexpr double kHistBin = 0.5;
    constexpr int kHistBins = 22;  // (kHistHi - kHistLo) / kHistBin
    auto logHistogram = [&](const char * label, const std::vector<Eigen::Vector3d> & pts) {
      if (pts.empty()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] DIAG zhist %s: n=0", name_.c_str(), label);
        return;
      }
      std::vector<double> zs;
      zs.reserve(pts.size());
      for (const auto & p : pts) zs.push_back(p.z());
      std::sort(zs.begin(), zs.end());
      auto percentile = [&](double frac) {
        std::size_t idx = static_cast<std::size_t>(frac * static_cast<double>(zs.size() - 1) + 0.5);
        idx = std::min(idx, zs.size() - 1);
        return zs[idx];
      };
      int counts[kHistBins] = {0};
      for (double z : zs) {
        if (z < kHistLo || z >= kHistHi) continue;
        int bin = static_cast<int>((z - kHistLo) / kHistBin);
        bin = std::clamp(bin, 0, kHistBins - 1);
        ++counts[bin];
      }
      std::string row1, row2;
      for (int b = 0; b < kHistBins; ++b) {
        if (counts[b] == 0) continue;
        const int pct =
          static_cast<int>(100.0 * static_cast<double>(counts[b]) / static_cast<double>(zs.size()) + 0.5);
        if (pct == 0) continue;
        const double lo = kHistLo + static_cast<double>(b) * kHistBin;
        std::string & row = (b < kHistBins / 2) ? row1 : row2;
        row += " " + formatHalf(lo) + ":" + std::to_string(pct);
      }
      RCLCPP_INFO(
        node_->get_logger(), "[%s] DIAG zhist %s: n=%zu p5=%.2f p50=%.2f p95=%.2f bins[z_lo:pct%%]:%s",
        name_.c_str(), label, zs.size(), percentile(0.05), percentile(0.50), percentile(0.95), row1.c_str());
      if (!row2.empty()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] DIAG zhist %s (cont'd):%s", name_.c_str(), label, row2.c_str());
      }
    };

    std::vector<Eigen::Vector3d> map_body;
    {
      const std::size_t hist_kf_count = std::min<std::size_t>(20, near_kf_bodies.size());
      std::size_t total = 0;
      for (std::size_t i = 0; i < hist_kf_count; ++i) total += near_kf_bodies[i].size();
      map_body.reserve(total);
      for (std::size_t i = 0; i < hist_kf_count; ++i) {
        map_body.insert(map_body.end(), near_kf_bodies[i].begin(), near_kf_bodies[i].end());
      }
    }
    logHistogram("scan", scan_body);
    logHistogram("map", map_body);

    // -----------------------------------------------------------------------------------------
    // Part 2: band discrimination sweep. All bounds are metres above the road surface (the body
    // frame's z=0), since base_link here is base_footprint -- see the block comment above.
    // -----------------------------------------------------------------------------------------
    struct BandCase
    {
      const char * label;
      double lo;
      double hi;
    };
    const BandCase kBands[] = {
      {"off", -1000.0, 1000.0},   // no band at all, road surface included
      {"current", 0.6, 6.0},      // today's default
      {"withground", -0.5, 6.0},  // adds the road surface back
      {"noground", 0.3, 20.0},    // strip only the road, keep all structure incl. canopy
      {"struct", 0.3, 3.0},       // strip road AND canopy: curbs, barriers, vehicles, walls, pole bases
      {"mid", 1.0, 4.0},
      {"canopy", 3.0, 10.0},  // CONTROL: canopy only. Expected to discriminate WORST.
    };
    auto inBand = [](double z, double lo, double hi) { return z >= lo && z <= hi; };

    const eidos::reloc::VoxelPyramid::Config cfg_local_base = [&] {
      eidos::reloc::VoxelPyramid::Config c = pyramid_.config();
      // build_free_space=false: the free-space channel is irrelevant to this diagnostic (it only
      // ever consults occupancy/distance-field hits) and would only cost extra build time.
      c.build_free_space = false;
      // max_height=0.0 (disabled): the whole point is a body-frame band applied by `inBand` below
      // BEFORE insertion, not a map-frame clamp applied inside the pyramid -- see buildPyramid()'s
      // identical reasoning for why cfg.max_height stays disabled on the real pyramid too.
      c.max_height = 0.0;
      return c;
    }();

    RCLCPP_INFO(node_->get_logger(), "[%s] DIAG band sweep: %zu nearby keyframes", name_.c_str(), near_kfs.size());

    for (const auto & band : kBands) {
      if (stop_requested_.load()) break;

      // 1. Local pyramid built ONLY from points in this band, from every nearby keyframe.
      eidos::reloc::VoxelPyramid local;
      local.beginInsert(cfg_local_base);
      for (std::size_t i = 0; i < near_kfs.size(); ++i) {
        const Eigen::Isometry3d & T = near_kfs[i].second;
        for (const auto & p : near_kf_bodies[i]) {
          if (inBand(p.z(), band.lo, band.hi)) local.insert(T * p);
        }
      }
      local.finalize();

      // 2. Query for this band: filter the (band-free) buffered scan by the band, then apply the
      // SAME strided downsample the real query builder (lidarCallback()) uses.
      std::vector<Eigen::Vector3d> band_query;
      band_query.reserve(scan_body.size());
      for (const auto & p : scan_body) {
        if (inBand(p.z(), band.lo, band.hi)) band_query.push_back(p);
      }
      std::vector<Eigen::Vector3d> bq;
      if (target_query_points_ > 0 && static_cast<int>(band_query.size()) > target_query_points_) {
        const std::size_t target = static_cast<std::size_t>(target_query_points_);
        const std::size_t n = band_query.size();
        const double step = static_cast<double>(n) / static_cast<double>(target);
        bq.reserve(target);
        for (std::size_t k = 0; k < target; ++k) {
          std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
          if (idx >= n) idx = n - 1;
          bq.push_back(band_query[idx]);
        }
      } else {
        bq = band_query;
      }

      if (bq.size() < 20 || local.empty()) {
        RCLCPP_INFO(
          node_->get_logger(), "[%s] DIAG band %s: degenerate (query=%zu map_voxels=%zu), skipped", name_.c_str(),
          band.label, bq.size(), local.level(0).voxels.size());
        local.releaseMemory();
        continue;
      }

      // YAW SWEEP: 36 yaws starting at the true probe yaw. i==0 is the true yaw itself, so its
      // rank among the 36 (1 = best) is the single number that decides whether this band lets the
      // scorer tell the correct heading apart from a wrong one.
      constexpr int kNumYaw = 36;
      double yaw_raws[kNumYaw];
      double true_raw = 0.0;
      int max_possible = 1;
      for (int i = 0; i < kNumYaw; ++i) {
        const double yaw_i = probe_yaw + static_cast<double>(i) * (2.0 * M_PI / static_cast<double>(kNumYaw));
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(
          local, bq, probe_t, yaw_i, 0, hit_weight_, local.effectiveScoreMode());
        yaw_raws[i] = static_cast<double>(bd.raw);
        if (i == 0) {
          true_raw = static_cast<double>(bd.raw);
          max_possible = std::max(1, bd.max_possible);
        }
      }
      int yaw_rank = 1;
      double yaw_sum = 0.0;
      for (int i = 0; i < kNumYaw; ++i) {
        yaw_sum += yaw_raws[i];
        if (i != 0 && yaw_raws[i] > true_raw) ++yaw_rank;
      }
      const double yaw_mean = yaw_sum / static_cast<double>(kNumYaw);
      const double yaw_ratio = yaw_mean > 0.0 ? true_raw / yaw_mean : 0.0;
      const double yaw_norm = true_raw / static_cast<double>(max_possible);

      // XY SWEEP: 9x9 grid of 1 m steps at the true yaw. (dx,dy)=(0,0) is the true translation.
      double true_xy_raw = 0.0;
      double xy_sum = 0.0;
      std::vector<double> xy_raws;
      xy_raws.reserve(81);
      for (int dxi = -4; dxi <= 4; ++dxi) {
        for (int dyi = -4; dyi <= 4; ++dyi) {
          const Eigen::Vector3d t =
            probe_t + Eigen::Vector3d(static_cast<double>(dxi), static_cast<double>(dyi), 0.0);
          const auto bd = eidos::reloc::scoreBreakdownAtLevel(
            local, bq, t, probe_yaw, 0, hit_weight_, local.effectiveScoreMode());
          const double raw = static_cast<double>(bd.raw);
          xy_raws.push_back(raw);
          xy_sum += raw;
          if (dxi == 0 && dyi == 0) true_xy_raw = raw;
        }
      }
      int xy_rank = 1;
      for (double raw : xy_raws) {
        if (raw > true_xy_raw) ++xy_rank;
      }
      const double xy_mean = xy_sum / static_cast<double>(xy_raws.size());
      const double xy_ratio = xy_mean > 0.0 ? true_xy_raw / xy_mean : 0.0;

      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] DIAG band %s z=[%.2f,%.2f] query=%zu map_voxels=%zu yaw_rank=%d/%d yaw_ratio=%.3f norm=%.3f "
        "xy_rank=%d/%d xy_ratio=%.3f",
        name_.c_str(), band.label, band.lo, band.hi, bq.size(), local.level(0).voxels.size(), yaw_rank, kNumYaw,
        yaw_ratio, yaw_norm, xy_rank, static_cast<int>(xy_raws.size()), xy_ratio);

      local.releaseMemory();
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] DIAG band sweep complete", name_.c_str());
  }

  // ===========================================================================================
  // TEMPORARY DIAGNOSTIC: resolution / scoring-mode / min-observation-count sweep.
  //
  // The band sweep above exonerated the height band: even its best case ("struct", 0.3-3.0 m)
  // only reached yaw_rank 10.8/36 against a random expectation of 18.5 (ratio 1.050), and the
  // currently configured default band was already the best of the seven tried. That leaves the
  // working hypothesis that the prior map is nearly SPACE-FILLING at the 1.0 m level-0 voxel
  // resolution: 992 keyframes of full-resolution Velodyne accumulate into ~1.1M level-0 voxels
  // over a driven corridor whose volume is of the same order, so "does this query point land in
  // an occupied voxel" is close to always-true regardless of pose, independent of which height
  // slice is asked. This sweep tests that directly along three axes at once: finer voxels (which
  // shrink each cell's footprint and so should un-saturate occupancy), a per-voxel MINIMUM
  // OBSERVATION COUNT (which turns a single stray point crossing a cell into "not really
  // occupied," suppressing the noise floor a 992-keyframe accumulation creates), and both scoring
  // modes (occupancy vs. distance field) so a resolution effect is not confused with a mode
  // effect.
  //
  // Critically, this sweep reports not just each case's RANK (as the band sweep did) but WHERE
  // the score actually peaks -- the argmax of the fine yaw sweep and of the fine xy sweep. Rank
  // alone cannot distinguish two very different failure modes that would otherwise look
  // identical: a bad rank because the score has no peak anywhere (truly uninformative, matching
  // the space-filling hypothesis), versus a bad rank because the score DOES peak, just not
  // exactly at the reference pose. The latter is plausible here: GICP polish from the reference
  // pose moved 0.58 m before converging, so the "ground truth" pose fed into this diagnostic may
  // itself be off by roughly that much. A rank far from 1 whose argmax sits CONSISTENTLY at a
  // specific nonzero offset means the score does peak and the reference pose is simply displaced
  // (GICP from the reference moved 0.58 m, so a real offset is plausible). A rank far from 1
  // whose argmax wanders run to run means there is genuinely no peak. That is the one distinction
  // this diagnostic exists to draw, and it is why fyaw_argmax/xy_argmax/z_argmax are logged
  // alongside every rank rather than the rank alone.
  //
  // This is NOT wired into the live search or scoring path in any way -- it is read-only
  // diagnostics, gated behind debug_res_sweep_ (default false) and debug_probe_pose_, and is
  // meant to be deleted once the question above is answered.
  // ===========================================================================================
  if (debug_res_sweep_ && debug_probe_pose_.size() >= 4) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
    const double probe_yaw = debug_probe_pose_[3] * M_PI / 180.0;

    // -----------------------------------------------------------------------------------------
    // Shared setup, duplicated (not hoisted) from the debug_band_sweep_ block immediately above:
    // hoisting would mean either merging the two `if` guards (so a user could no longer run one
    // sweep without the other) or lifting these locals out to function scope (extra risk of
    // accidentally changing what the band-sweep block reads), and this diagnostic is temporary
    // and read-only either way, so the small duplication is the lower-risk choice. Semantics are
    // identical to the block above: de-tilted, range-filtered buffered scan (band NOT yet
    // applied), and prior-map keyframes within 100 m of the probe pose.
    // -----------------------------------------------------------------------------------------
    small_gicp::PointCloud::Ptr scan_copy;
    {
      std::lock_guard<std::mutex> lock(scan_lock_);
      scan_copy = latest_scan_;
    }
    double roll = 0.0, pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(imu_lock_);
      roll = latest_imu_roll_;
      pitch = latest_imu_pitch_;
    }
    const Eigen::Matrix3d r_detilt = rotYX(pitch, roll);
    const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
    std::vector<Eigen::Vector3d> scan_body;
    if (scan_copy) {
      scan_body.reserve(scan_copy->size());
      for (std::size_t i = 0; i < scan_copy->size(); ++i) {
        const Eigen::Vector3d p = r_detilt * scan_copy->point(i).head<3>();
        if (max_range_sq > 0.0 && p.head<2>().squaredNorm() > max_range_sq) continue;
        scan_body.push_back(p);
      }
    }

    std::vector<std::pair<gtsam::Key, Eigen::Isometry3d>> near_kfs;
    {
      auto key_list = map_manager_->getKeyList();
      for (gtsam::Key key : key_list) {
        if (!map_manager_->isPriorMapKey(key)) continue;
        int idx = map_manager_->getCloudIndex(key);
        if (idx < 0 || static_cast<std::size_t>(idx) >= poses6d_->points.size()) continue;
        Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(idx)]);
        Eigen::Isometry3d T;
        T.matrix() = world_t.matrix().cast<double>();
        if ((T.translation().head<2>() - probe_t.head<2>()).norm() > 100.0) continue;
        near_kfs.emplace_back(key, T);
      }
    }

    const std::string cloud_suffix = "/cloud";
    const bool has_fallback = pointcloud_from_.size() >= cloud_suffix.size() &&
      pointcloud_from_.compare(pointcloud_from_.size() - cloud_suffix.size(), cloud_suffix.size(), cloud_suffix) ==
        0;
    const std::string fallback_key = has_fallback
      ? pointcloud_from_.substr(0, pointcloud_from_.size() - cloud_suffix.size()) + "/gicp_cloud"
      : std::string();
    auto retrieveBody = [&](gtsam::Key key, std::vector<Eigen::Vector3d> & out) -> bool {
      auto tryKey = [&](const std::string & data_key) -> bool {
        auto pcl_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(key, data_key);
        if (pcl_opt.has_value() && *pcl_opt && !(*pcl_opt)->empty()) {
          out.reserve(out.size() + (*pcl_opt)->points.size());
          for (const auto & pt : (*pcl_opt)->points) out.emplace_back(pt.x, pt.y, pt.z);
          return true;
        }
        auto gicp_opt = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(key, data_key);
        if (gicp_opt.has_value() && *gicp_opt && !(*gicp_opt)->empty()) {
          out.reserve(out.size() + (*gicp_opt)->size());
          for (std::size_t i = 0; i < (*gicp_opt)->size(); ++i) {
            const Eigen::Vector3d p = (*gicp_opt)->point(i).head<3>();
            out.push_back(p);
          }
          return true;
        }
        return false;
      };
      if (has_fallback && prefer_downsampled_source_) {
        if (tryKey(fallback_key)) return true;
        return tryKey(pointcloud_from_);
      }
      if (tryKey(pointcloud_from_)) return true;
      return has_fallback && tryKey(fallback_key);
    };

    // Retrieve every near keyframe's RAW body-frame points once, then keep only the ones passing
    // the plugin's ACTUAL configured height band (inHeightBand()) -- the same band the real
    // pyramid rasterizes with -- alongside which keyframe each surviving point came from. This
    // flat list is what every case's Step 1 (below) folds into a per-resolution voxel count
    // without re-retrieving or re-filtering per case, since neither retrieval nor the height
    // filter depends on the case's resolution.
    std::vector<std::pair<std::size_t, Eigen::Vector3d>> band_pts;
    {
      std::vector<Eigen::Vector3d> kf_body;
      for (std::size_t i = 0; i < near_kfs.size(); ++i) {
        kf_body.clear();
        retrieveBody(near_kfs[i].first, kf_body);
        for (const auto & p : kf_body) {
          if (inHeightBand(p.z())) band_pts.emplace_back(i, p);
        }
      }
    }

    // Query set: built ONCE, since it does not depend on the case (resolution/mode/min_points all
    // only affect the MAP side, never the query). Same semantics as the real query builder
    // (lidarCallback()): band-filter the de-tilted, range-filtered scan, then apply the identical
    // strided downsample to target_query_points_.
    std::vector<Eigen::Vector3d> q;
    {
      std::vector<Eigen::Vector3d> filtered;
      filtered.reserve(scan_body.size());
      for (const auto & p : scan_body) {
        if (inHeightBand(p.z())) filtered.push_back(p);
      }
      if (target_query_points_ > 0 && static_cast<int>(filtered.size()) > target_query_points_) {
        const std::size_t target = static_cast<std::size_t>(target_query_points_);
        const std::size_t n = filtered.size();
        const double step = static_cast<double>(n) / static_cast<double>(target);
        q.reserve(target);
        for (std::size_t k = 0; k < target; ++k) {
          std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
          if (idx >= n) idx = n - 1;
          q.push_back(filtered[idx]);
        }
      } else {
        q = std::move(filtered);
      }
    }

    // Case table. Three axes varied together: resolution (coarser -> finer), scoring mode
    // (occupancy vs. distance field), and a per-voxel minimum observation count (mp) that treats
    // a map voxel as occupied only once at least that many source points landed in it -- the
    // direct test of "is a single stray point across 992 keyframes enough to saturate a cell."
    // sigma/trunc are meaningless (and ignored by the pyramid) whenever distance_field is false.
    struct ResCase
    {
      const char * label;
      double res;           // min_voxel_size for the local pyramid
      double sigma;         // df_sigma (ignored under occupancy)
      int trunc;             // df_truncation_voxels (ignored under occupancy)
      bool distance_field;  // true => ScoreMode::DistanceField, false => ScoreMode::Occupancy
      int min_points;        // keep a voxel only if >= this many source points fall in it
    };
    const ResCase kCases[] = {
      {"base_df_1.0",   1.00, 1.00, 2, true,   1},
      {"df_1.0_mp3",    1.00, 1.00, 2, true,   3},
      {"df_1.0_mp10",   1.00, 1.00, 2, true,  10},
      {"df_1.0_mp30",   1.00, 1.00, 2, true,  30},
      {"occ_1.0",       1.00, 0.00, 0, false,  1},
      {"occ_1.0_mp10",  1.00, 0.00, 0, false, 10},
      {"df_0.5",        0.50, 0.50, 3, true,   1},
      {"df_0.5_mp10",   0.50, 0.50, 3, true,  10},
      {"occ_0.5",       0.50, 0.00, 0, false,  1},
      {"df_0.25_s0.25", 0.25, 0.25, 4, true,   1},
      {"df_0.25_s0.5",  0.25, 0.50, 4, true,   1},
      {"occ_0.25",      0.25, 0.00, 0, false,  1},
      // Added after a measurement showed the true pose scoring BELOW ambient density at 1 m
      // containment (36% at truth vs. 48% at a 60 deg-wrong yaw) -- the signature of a systematic
      // query-to-map offset that lands the query just off surfaces, in the thin free layer beside
      // them. Only a resolution fine enough to resolve a sub-metre offset can show that as a real
      // peak rather than noise, which is what these two cases (plus the existing 0.25 m cases
      // above) are for -- df_0.25_s0.15 tightens truncation to well below the map's
      // inter-structure spacing, and occ_0.25_mp5 is the fine grid WITH observation thresholding,
      // to check the two effects (resolution, and denoising by count) are separable.
      {"df_0.25_s0.15", 0.25, 0.15, 2, true,   1},   // 0.5 m truncation
      {"occ_0.25_mp5",  0.25, 0.00, 0, false,  5},   // fine AND observation-thresholded
    };

    RCLCPP_INFO(
      node_->get_logger(), "[%s] DIAG res sweep: %zu nearby keyframes, %zu band points, query=%zu", name_.c_str(),
      near_kfs.size(), band_pts.size(), q.size());

    for (const auto & rc : kCases) {
      if (stop_requested_.load()) break;

      // Step 1: per-voxel observation counts at THIS case's resolution, in world frame. This is
      // recomputed per case (unlike band_pts/q above) because the voxel key -- and therefore
      // which points fall in the same cell -- depends on `rc.res`.
      std::unordered_map<int64_t, int> counts;
      counts.reserve(band_pts.size() / 4 + 16);
      const double inv_res = 1.0 / rc.res;
      for (const auto & [kf_idx, p] : band_pts) {
        const Eigen::Isometry3d & T = near_kfs[kf_idx].second;
        const Eigen::Vector3d wp = T * p;
        const int64_t ix = eidos::reloc::voxelIndex(wp.x(), inv_res);
        const int64_t iy = eidos::reloc::voxelIndex(wp.y(), inv_res);
        const int64_t iz = eidos::reloc::voxelIndex(wp.z(), inv_res);
        ++counts[eidos::reloc::packVoxel(ix, iy, iz)];
      }

      // Step 2: local pyramid, ONE point per surviving voxel (its centre), not the raw
      // accumulation. Deliberate: the occupancy set is identical either way, and for the distance
      // field this makes the field's source the thresholded structure rather than the raw point
      // pile, which is exactly the quantity under test here (does thresholding by observation
      // count change what the scorer sees).
      eidos::reloc::VoxelPyramid::Config cfg_case = pyramid_.config();
      cfg_case.min_voxel_size = rc.res;
      cfg_case.num_levels = 1;  // only level 0 is ever scored below; coarser levels would only
                                 // cost build time, especially at the 0.25 m cases.
      cfg_case.build_free_space = false;
      cfg_case.max_height = 0.0;  // band already applied in body frame, above (band_pts).
      cfg_case.score_mode =
        rc.distance_field ? eidos::reloc::ScoreMode::DistanceField : eidos::reloc::ScoreMode::Occupancy;
      cfg_case.df_sigma = rc.sigma;
      cfg_case.df_truncation_voxels = rc.trunc;
      // cfg_case.max_score_voxels left at pyramid_.config()'s value -- the guard that lets a
      // 0.25 m distance-field case abandon gracefully (see distanceFieldAbandoned() check below)
      // instead of exhausting memory, exactly like the real pyramid build.

      eidos::reloc::VoxelPyramid local;
      local.beginInsert(cfg_case);
      for (const auto & kv : counts) {
        if (kv.second < rc.min_points) continue;
        int64_t ix, iy, iz;
        eidos::reloc::unpackVoxel(kv.first, ix, iy, iz);
        local.insert(Eigen::Vector3d(
          (static_cast<double>(ix) + 0.5) * rc.res, (static_cast<double>(iy) + 0.5) * rc.res,
          (static_cast<double>(iz) + 0.5) * rc.res));
      }
      local.finalize();
      const eidos::reloc::ScoreMode effective_mode = local.effectiveScoreMode();

      // Step 4: COARSE yaw sweep, 36 yaws starting at the true probe yaw (i==0). This is the same
      // rank statistic the band sweep computed -- kept for continuity/comparability with those
      // numbers -- but is not, on its own, enough to interpret a bad rank (see the block comment
      // above): that is what the fine sweeps and their argmax are for.
      constexpr int kNumYaw = 36;
      double yaw_raws[kNumYaw];
      double true_yaw_raw = 0.0;
      int max_possible = 1;
      for (int i = 0; i < kNumYaw; ++i) {
        const double yaw_i = probe_yaw + static_cast<double>(i) * (2.0 * M_PI / static_cast<double>(kNumYaw));
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(local, q, probe_t, yaw_i, 0, hit_weight_, effective_mode);
        yaw_raws[i] = static_cast<double>(bd.raw);
        if (i == 0) {
          true_yaw_raw = static_cast<double>(bd.raw);
          max_possible = std::max(1, bd.max_possible);
        }
      }
      int yaw_rank = 1;
      double yaw_sum = 0.0;
      for (int i = 0; i < kNumYaw; ++i) {
        yaw_sum += yaw_raws[i];
        if (i != 0 && yaw_raws[i] > true_yaw_raw) ++yaw_rank;
      }
      const double yaw_mean = yaw_sum / static_cast<double>(kNumYaw);
      const double yaw_ratio = yaw_mean > 0.0 ? true_yaw_raw / yaw_mean : 0.0;
      const double norm = true_yaw_raw / static_cast<double>(max_possible);

      // Step 5: FINE yaw sweep, 1-degree steps over +/-20 deg around the true yaw. fyaw_argmax is
      // the point of this sweep (see the block comment above): a peak that consistently lands at
      // the same nonzero degree offset across cases means the score IS discriminating heading,
      // just not centred exactly on the (possibly slightly wrong) reference; a peak that wanders
      // means the score has no real relationship to heading at all.
      constexpr int kNumFineYaw = 41;  // d = -20..20 inclusive
      double fyaw_raws[kNumFineYaw];
      double fyaw_true_raw = 0.0;
      double fyaw_max_raw = -1.0;
      int fyaw_argmax_d = 0;
      for (int idx = 0; idx < kNumFineYaw; ++idx) {
        const int d = idx - 20;
        const double yaw_i = probe_yaw + static_cast<double>(d) * (M_PI / 180.0);
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(local, q, probe_t, yaw_i, 0, hit_weight_, effective_mode);
        fyaw_raws[idx] = static_cast<double>(bd.raw);
        if (d == 0) fyaw_true_raw = fyaw_raws[idx];
        if (fyaw_raws[idx] > fyaw_max_raw) {
          fyaw_max_raw = fyaw_raws[idx];
          fyaw_argmax_d = d;
        }
      }
      int fyaw_rank = 1;
      for (int idx = 0; idx < kNumFineYaw; ++idx) {
        const int d = idx - 20;
        if (d != 0 && fyaw_raws[idx] > fyaw_true_raw) ++fyaw_rank;
      }

      // Step 6: FINE xy sweep, 17x17 grid at 0.5 m steps over +/-4.0 m, at the true yaw.
      // xy_argmax is the second half of the point of this diagnostic: a peak sitting consistently
      // at the same nonzero (dx,dy) is the signature of a displaced-but-real reference pose (GICP
      // from the reference moved 0.58 m before converging, so an offset of roughly that size is
      // plausible); a peak that wanders run to run means there is no spatial structure to find.
      constexpr int kNumXY = 17;  // -4.0 .. 4.0 in 0.5 m steps
      std::vector<double> xy_raws;
      xy_raws.reserve(static_cast<std::size_t>(kNumXY) * static_cast<std::size_t>(kNumXY));
      double xy_true_raw = 0.0;
      double xy_max_raw = -1.0;
      double xy_argmax_dx = 0.0;
      double xy_argmax_dy = 0.0;
      for (int i = 0; i < kNumXY; ++i) {
        const double dx = -4.0 + static_cast<double>(i) * 0.5;
        for (int j = 0; j < kNumXY; ++j) {
          const double dy = -4.0 + static_cast<double>(j) * 0.5;
          const Eigen::Vector3d t = probe_t + Eigen::Vector3d(dx, dy, 0.0);
          const auto bd = eidos::reloc::scoreBreakdownAtLevel(local, q, t, probe_yaw, 0, hit_weight_, effective_mode);
          const double raw = static_cast<double>(bd.raw);
          xy_raws.push_back(raw);
          if (dx == 0.0 && dy == 0.0) xy_true_raw = raw;
          if (raw > xy_max_raw) {
            xy_max_raw = raw;
            xy_argmax_dx = dx;
            xy_argmax_dy = dy;
          }
        }
      }
      int xy_rank = 1;
      for (double raw : xy_raws) {
        if (raw > xy_true_raw) ++xy_rank;
      }

      // Step 7: z sweep, 0.5 m steps over +/-3.0 m, at the true (x,y,yaw). Same argmax logic as
      // xy/yaw above, applied to height -- a real vertical registration offset would show up here
      // as a consistent nonzero z_argmax rather than a wandering one.
      constexpr int kNumZ = 13;  // -3.0 .. 3.0 in 0.5 m steps
      std::vector<double> z_raws;
      z_raws.reserve(kNumZ);
      double z_true_raw = 0.0;
      double z_max_raw = -1.0;
      double z_argmax_dz = 0.0;
      for (int i = 0; i < kNumZ; ++i) {
        const double dz = -3.0 + static_cast<double>(i) * 0.5;
        const Eigen::Vector3d t = probe_t + Eigen::Vector3d(0.0, 0.0, dz);
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(local, q, t, probe_yaw, 0, hit_weight_, effective_mode);
        const double raw = static_cast<double>(bd.raw);
        z_raws.push_back(raw);
        if (dz == 0.0) z_true_raw = raw;
        if (raw > z_max_raw) {
          z_max_raw = raw;
          z_argmax_dz = dz;
        }
      }
      int z_rank = 1;
      for (double raw : z_raws) {
        if (raw > z_true_raw) ++z_rank;
      }

      // Step 8: one log line per case. df_abandoned=yes means the pyramid's own max_score_voxels
      // guard fired while building THIS case's distance field (see VoxelPyramid::finalize() /
      // distanceFieldAbandoned()) and it silently fell back to occupancy scoring -- reported
      // explicitly here (rather than left implicit) so occupancy-shaped numbers under a
      // "distance_field" case label are never mistaken for an actual distance-field result.
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] DIAG res %s res=%.2f mode=%s sigma=%.2f mp=%d map_vox=%zu raw_vox=%zu query=%zu yaw_rank=%d/%d "
        "yaw_ratio=%.3f fyaw_rank=%d/%d fyaw_argmax=%+ddeg xy_rank=%d/%zu xy_argmax=(%.2f,%.2f) z_rank=%d/%d "
        "z_argmax=%.2f norm=%.3f df_abandoned=%s",
        name_.c_str(), rc.label, rc.res, rc.distance_field ? "df" : "occ", rc.sigma, rc.min_points,
        local.level(0).voxels.size(), counts.size(), q.size(), yaw_rank, kNumYaw, yaw_ratio, fyaw_rank, kNumFineYaw,
        fyaw_argmax_d, xy_rank, xy_raws.size(), xy_argmax_dx, xy_argmax_dy, z_rank, kNumZ, z_argmax_dz, norm,
        local.distanceFieldAbandoned() ? "yes" : "no");

      // Step 9: release this case's pyramid before building the next one -- these are local,
      // scoped VoxelPyramid instances distinct from the search's real pyramid_ member, but the
      // 0.25 m distance-field cases in particular can still be multi-hundred-MB, and there is no
      // reason to hold two of them live at once.
      local.releaseMemory();
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] DIAG res sweep complete", name_.c_str());
  }

  // ===========================================================================================
  // TEMPORARY DIAGNOSTIC: self-test control -- score a prior-map keyframe's OWN cloud at its
  // OWN pose.
  //
  // Every measurement made by the debug_band_sweep_ and debug_res_sweep_ blocks above ran the
  // LIVE scan through the full acquisition path first: TF extrinsics from the URDF (possibly not
  // the calibration this map was built with), IMU de-tilt, downsampling, and the height band.
  // None of those diagnostics -- however many bands, resolutions, scoring modes and observation
  // thresholds they swept -- can tell apart two very different explanations for the same bad
  // rank: "the scene truly has no x/y/yaw signal at this resolution" versus "the live-query path
  // is corrupting an otherwise-good signal before it ever reaches the scorer." This block
  // resolves that by removing every live-path suspect at once: the query here is a prior-map
  // keyframe's own body-frame cloud, de-tilted by ITS OWN recovered roll/pitch and scored at ITS
  // OWN recorded pose -- so it is exactly, trivially registered to the map by construction, free
  // of calibration error, sensor differences, dynamic objects and IMU de-tilt error.
  //
  // C1 tests the PLUMBING: the self-query is scored against the pyramid that ALREADY INCLUDES
  // this keyframe, so a bad result (exact hit far below 100%, yaw_rank far from 1/36) can only
  // mean a transform or scoring bug somewhere in this diagnostic itself (or, by extension, in
  // the shared scoring path it exercises) -- because the query points are, quite literally, a
  // subset of the map being searched.
  //
  // C2/C3 test the SCENE: the self-query is scored against a LOCAL map that has the keyframe
  // (and everything within kExcludeRadius of it) surgically removed, so the query can no longer
  // trivially match itself -- this is the honest single-scan localization test for this route. A
  // bad result here, with C1 passing cleanly, means this scene genuinely lacks single-scan x/y/
  // yaw observability at the tested resolution: no plumbing bug to blame, no live-path corruption
  // to blame, just an intrinsically hard (e.g. corridor-shaped, repetitive) piece of geometry.
  // C1 vs. C2/C3 is precisely the pair that separates those two explanations.
  //
  // This is NOT wired into the live search or scoring path in any way -- it is read-only
  // diagnostics, gated behind debug_self_test_ (default false) and debug_probe_pose_, and is
  // meant to be deleted once the question above is answered.
  // ===========================================================================================
  if (debug_self_test_ && debug_probe_pose_.size() >= 4) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);

    // Same PCL-typed-then-small_gicp-typed retrieval, and source-key selection, that
    // insertKeyframeCloud()/buildPyramid() and the two sweep blocks above use. Returns
    // BODY-frame points -- not transformed by a keyframe's world pose, no height band applied.
    const std::string cloud_suffix = "/cloud";
    const bool has_fallback = pointcloud_from_.size() >= cloud_suffix.size() &&
      pointcloud_from_.compare(pointcloud_from_.size() - cloud_suffix.size(), cloud_suffix.size(), cloud_suffix) ==
        0;
    const std::string fallback_key = has_fallback
      ? pointcloud_from_.substr(0, pointcloud_from_.size() - cloud_suffix.size()) + "/gicp_cloud"
      : std::string();
    auto retrieveBody = [&](gtsam::Key key, std::vector<Eigen::Vector3d> & out) -> bool {
      auto tryKey = [&](const std::string & data_key) -> bool {
        auto pcl_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(key, data_key);
        if (pcl_opt.has_value() && *pcl_opt && !(*pcl_opt)->empty()) {
          out.reserve(out.size() + (*pcl_opt)->points.size());
          for (const auto & pt : (*pcl_opt)->points) out.emplace_back(pt.x, pt.y, pt.z);
          return true;
        }
        auto gicp_opt = map_manager_->retrieve<small_gicp::PointCloud::Ptr>(key, data_key);
        if (gicp_opt.has_value() && *gicp_opt && !(*gicp_opt)->empty()) {
          out.reserve(out.size() + (*gicp_opt)->size());
          for (std::size_t i = 0; i < (*gicp_opt)->size(); ++i) {
            const Eigen::Vector3d p = (*gicp_opt)->point(i).head<3>();
            out.push_back(p);
          }
          return true;
        }
        return false;
      };
      if (has_fallback && prefer_downsampled_source_) {
        if (tryKey(fallback_key)) return true;
        return tryKey(pointcloud_from_);
      }
      if (tryKey(pointcloud_from_)) return true;
      return has_fallback && tryKey(fallback_key);
    };

    // Mirrors the live query builder's transform chain (lidarCallback()) -- the same filter and
    // downsample logic the new buildSelfQuery() member function applies internally. Kept here as
    // a local lambda (renamed to avoid shadowing the member function of the same purpose) only
    // because C3 below selects its second keyframe by TRAJECTORY POSITION rather than nearest
    // point, so it cannot use buildSelfQuery()'s nearest-keyframe selection; C1/C2 use
    // buildSelfQuery() directly instead of this lambda. De-tilt by rotYX(pitch, roll), drop
    // points beyond max_query_range_ horizontally, keep only the configured height band, then
    // apply the identical strided downsample to target_query_points_.
    auto filterSelfCloud = [&](const std::vector<Eigen::Vector3d> & body_pts, double pitch, double roll) {
      const Eigen::Matrix3d r_detilt = rotYX(pitch, roll);
      const double max_range_sq = max_query_range_ > 0.0 ? max_query_range_ * max_query_range_ : 0.0;
      std::vector<Eigen::Vector3d> filtered;
      filtered.reserve(body_pts.size());
      for (const auto & p : body_pts) {
        const Eigen::Vector3d pd = r_detilt * p;
        if (max_range_sq > 0.0 && pd.head<2>().squaredNorm() > max_range_sq) continue;
        if (!inHeightBand(pd.z())) continue;
        filtered.push_back(pd);
      }
      std::vector<Eigen::Vector3d> out;
      if (target_query_points_ > 0 && static_cast<int>(filtered.size()) > target_query_points_) {
        const std::size_t target = static_cast<std::size_t>(target_query_points_);
        const std::size_t n = filtered.size();
        const double step = static_cast<double>(n) / static_cast<double>(target);
        out.reserve(target);
        for (std::size_t k = 0; k < target; ++k) {
          std::size_t idx = static_cast<std::size_t>(static_cast<double>(k) * step);
          if (idx >= n) idx = n - 1;
          out.push_back(filtered[idx]);
        }
      } else {
        out = std::move(filtered);
      }
      return out;
    };

    // Per-control metrics: exact level-0 occupancy hit fraction at the reference pose (computed
    // directly via VoxelLevel::hit(), so it is meaningful regardless of the active scoring
    // mode), plus the same coarse-yaw/fine-yaw/fine-xy rank+argmax statistics the two sweeps
    // above report, computed against whichever pyramid the caller passes in.
    struct SelfMetrics
    {
      double exact_hit_pct = 0.0;
      int yaw_rank = 1;
      double yaw_ratio = 0.0;
      int fyaw_rank = 1;
      int fyaw_argmax = 0;
      int xy_rank = 1;
      double xy_argmax_dx = 0.0;
      double xy_argmax_dy = 0.0;
    };
    auto scoreSelf = [&](
                       const eidos::reloc::VoxelPyramid & pyr, const std::vector<Eigen::Vector3d> & q,
                       const Eigen::Vector3d & t_ref, double yaw_ref) -> SelfMetrics {
      SelfMetrics m;
      if (q.empty() || pyr.empty()) return m;
      const eidos::reloc::ScoreMode mode = pyr.effectiveScoreMode();

      // Exact hit fraction: deliberately NOT scoreBreakdownAtLevel()'s `hits` (which means
      // different things in the two modes -- see ScoreBreakdown's doc comment) but the raw
      // VoxelLevel::hit() containment test, so this one number means the same thing under
      // either scoring mode: "does the query point literally land in an occupied level-0
      // voxel."
      {
        const double c = std::cos(yaw_ref);
        const double s = std::sin(yaw_ref);
        int hits = 0;
        for (const auto & qp : q) {
          const Eigen::Vector3d p(
            c * qp.x() - s * qp.y() + t_ref.x(), s * qp.x() + c * qp.y() + t_ref.y(), qp.z() + t_ref.z());
          if (pyr.level(0).hit(p)) ++hits;
        }
        m.exact_hit_pct = 100.0 * static_cast<double>(hits) / static_cast<double>(q.size());
      }

      // Coarse yaw sweep: 36 yaws at 10 deg steps starting at the true reference yaw (i==0).
      constexpr int kNumYaw = 36;
      double yaw_raws[kNumYaw];
      double true_raw = 0.0;
      for (int i = 0; i < kNumYaw; ++i) {
        const double yaw_i = yaw_ref + static_cast<double>(i) * (2.0 * M_PI / static_cast<double>(kNumYaw));
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(pyr, q, t_ref, yaw_i, 0, hit_weight_, mode);
        yaw_raws[i] = static_cast<double>(bd.raw);
        if (i == 0) true_raw = yaw_raws[i];
      }
      double yaw_sum = 0.0;
      for (int i = 0; i < kNumYaw; ++i) {
        yaw_sum += yaw_raws[i];
        if (i != 0 && yaw_raws[i] > true_raw) ++m.yaw_rank;
      }
      const double yaw_mean = yaw_sum / static_cast<double>(kNumYaw);
      m.yaw_ratio = yaw_mean > 0.0 ? true_raw / yaw_mean : 0.0;

      // Fine yaw sweep: 41 yaws at 1 deg steps over +/-20 deg around the reference yaw.
      constexpr int kNumFineYaw = 41;  // d = -20..20 inclusive
      double fyaw_raws[kNumFineYaw];
      double fyaw_true_raw = 0.0;
      double fyaw_max_raw = -1.0;
      for (int idx = 0; idx < kNumFineYaw; ++idx) {
        const int d = idx - 20;
        const double yaw_i = yaw_ref + static_cast<double>(d) * (M_PI / 180.0);
        const auto bd = eidos::reloc::scoreBreakdownAtLevel(pyr, q, t_ref, yaw_i, 0, hit_weight_, mode);
        fyaw_raws[idx] = static_cast<double>(bd.raw);
        if (d == 0) fyaw_true_raw = fyaw_raws[idx];
        if (fyaw_raws[idx] > fyaw_max_raw) {
          fyaw_max_raw = fyaw_raws[idx];
          m.fyaw_argmax = d;
        }
      }
      for (int idx = 0; idx < kNumFineYaw; ++idx) {
        const int d = idx - 20;
        if (d != 0 && fyaw_raws[idx] > fyaw_true_raw) ++m.fyaw_rank;
      }

      // Fine xy sweep: 17x17 grid at 0.5 m steps over +/-4.0 m, at the reference yaw.
      constexpr int kNumXY = 17;  // -4.0 .. 4.0 in 0.5 m steps
      std::vector<double> xy_raws;
      xy_raws.reserve(static_cast<std::size_t>(kNumXY) * static_cast<std::size_t>(kNumXY));
      double xy_true_raw = 0.0;
      double xy_max_raw = -1.0;
      for (int i = 0; i < kNumXY; ++i) {
        const double dx = -4.0 + static_cast<double>(i) * 0.5;
        for (int j = 0; j < kNumXY; ++j) {
          const double dy = -4.0 + static_cast<double>(j) * 0.5;
          const Eigen::Vector3d t = t_ref + Eigen::Vector3d(dx, dy, 0.0);
          const auto bd = eidos::reloc::scoreBreakdownAtLevel(pyr, q, t, yaw_ref, 0, hit_weight_, mode);
          const double raw = static_cast<double>(bd.raw);
          xy_raws.push_back(raw);
          if (dx == 0.0 && dy == 0.0) xy_true_raw = raw;
          if (raw > xy_max_raw) {
            xy_max_raw = raw;
            m.xy_argmax_dx = dx;
            m.xy_argmax_dy = dy;
          }
        }
      }
      for (double raw : xy_raws) {
        if (raw > xy_true_raw) ++m.xy_rank;
      }

      return m;
    };

    // Build a LOCAL, single-level VoxelPyramid from every prior-map keyframe within 100 m
    // (horizontally) of `center`, EXCEPT those within kExcludeRadius of `exclude_pos` -- the
    // leave-one-out map C2/C3 score against. Same config pattern the two sweeps above use: 1
    // level, no free space, no map-frame height clamp (the band is applied in body frame below,
    // exactly like insertKeyframeCloud()/buildPyramid() do).
    constexpr double kExcludeRadius = 3.0;
    auto buildLocalMap = [&](
                           const Eigen::Vector3d & center, const Eigen::Vector3d & exclude_pos,
                           std::size_t & included, std::size_t & excluded) {
      eidos::reloc::VoxelPyramid::Config cfg = pyramid_.config();
      cfg.num_levels = 1;
      cfg.build_free_space = false;
      cfg.max_height = 0.0;
      eidos::reloc::VoxelPyramid local;
      local.beginInsert(cfg);
      included = 0;
      excluded = 0;
      auto key_list = map_manager_->getKeyList();
      for (gtsam::Key key : key_list) {
        if (stop_requested_.load()) break;
        if (!map_manager_->isPriorMapKey(key)) continue;
        int idx = map_manager_->getCloudIndex(key);
        if (idx < 0 || static_cast<std::size_t>(idx) >= poses6d_->points.size()) continue;
        Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(idx)]);
        Eigen::Isometry3d T;
        T.matrix() = world_t.matrix().cast<double>();
        if ((T.translation().head<2>() - center.head<2>()).norm() > 100.0) continue;
        if ((T.translation() - exclude_pos).norm() <= kExcludeRadius) {
          ++excluded;
          continue;
        }
        std::vector<Eigen::Vector3d> body;
        retrieveBody(key, body);
        for (const auto & p : body) {
          if (inHeightBand(p.z())) local.insert(T * p);
        }
        ++included;
      }
      local.finalize();
      return local;
    };

    // -----------------------------------------------------------------------------------------
    // Select the reference keyframe and build the self-query: the prior-map keyframe whose
    // position is closest to probe_t, de-tilted/filtered/downsampled exactly like the live query
    // path. Delegates to buildSelfQuery() (see its doc comment in the header) -- this used to be
    // inlined here (nearest-keyframe search, rpy decomposition + rotation-convention check,
    // cloud retrieval, de-tilt/filter/downsample), but debug_use_self_query_ in workerMain()
    // needs the exact same pipeline to substitute a self-query into the REAL search, so it is now
    // a shared member function instead of duplicated logic.
    // -----------------------------------------------------------------------------------------
    std::vector<Eigen::Vector3d> selfq;
    Eigen::Vector3d t_ref = Eigen::Vector3d::Zero();
    double yaw_ref = 0.0;
    int kf_idx = -1;
    const bool have_kf = buildSelfQuery(probe_t, selfq, t_ref, yaw_ref, kf_idx);

    if (!have_kf) {
      RCLCPP_WARN(node_->get_logger(), "[%s] DIAG self test: no prior-map keyframe found, skipping", name_.c_str());
    } else {
      const double best_d = (t_ref - probe_t).norm();
      RCLCPP_INFO(
        node_->get_logger(), "[%s] DIAG self kf=%d pos=(%.1f,%.1f,%.1f) d_probe=%.2fm", name_.c_str(), kf_idx,
        t_ref.x(), t_ref.y(), t_ref.z(), best_d);

      // ---------------------------------------------------------------------------------------
      // C1 -- PLUMBING control: score the self-query against the pyramid that ALREADY INCLUDES
      // this keyframe. The query points are a subset of the map being searched, transformed by
      // the SAME roll/pitch/yaw the map itself was built with, so the exact hit fraction must be
      // ~100% and yaw_rank must land at 1/36 with a large ratio. Anything less means there is a
      // transform or scoring bug in this diagnostic (and, by extension, in the shared scoring
      // path it exercises) -- report that as the single most important finding.
      // ---------------------------------------------------------------------------------------
      if (selfq.size() < 20 || pyramid_.empty()) {
        RCLCPP_WARN(
          node_->get_logger(), "[%s] DIAG self C1: degenerate (query=%zu, pyramid empty=%d), skipped", name_.c_str(),
          selfq.size(), pyramid_.empty() ? 1 : 0);
      } else {
        const SelfMetrics c1 = scoreSelf(pyramid_, selfq, t_ref, yaw_ref);
        RCLCPP_INFO(
          node_->get_logger(),
          "[%s] DIAG self C1 kf=%d d_probe=%.2fm query=%zu exact_hit=%.1f%% yaw_rank=%d/36 yaw_ratio=%.3f "
          "fyaw_rank=%d/41 fyaw_argmax=%+ddeg xy_rank=%d/289 xy_argmax=(%.2f,%.2f)",
          name_.c_str(), kf_idx, best_d, selfq.size(), c1.exact_hit_pct, c1.yaw_rank, c1.yaw_ratio, c1.fyaw_rank,
          c1.fyaw_argmax, c1.xy_rank, c1.xy_argmax_dx, c1.xy_argmax_dy);
      }

      // ---------------------------------------------------------------------------------------
      // C2 -- SCENE control: score the SAME self-query against a local map that EXCLUDES this
      // keyframe and everything within kExcludeRadius of it. The query can no longer trivially
      // match itself, so this is the honest single-scan localization test for this location: a
      // perfect, calibration-free query against a map that does not contain it. A bad rank here,
      // with C1 clean, means this scene genuinely lacks single-scan x/y/yaw observability.
      // ---------------------------------------------------------------------------------------
      std::size_t c2_included = 0, c2_excluded = 0;
      eidos::reloc::VoxelPyramid local2 = buildLocalMap(t_ref, t_ref, c2_included, c2_excluded);
      if (selfq.size() < 20 || local2.empty()) {
        RCLCPP_WARN(
          node_->get_logger(), "[%s] DIAG self C2: degenerate (query=%zu, included_kf=%zu, excluded_kf=%zu), skipped",
          name_.c_str(), selfq.size(), c2_included, c2_excluded);
      } else {
        const SelfMetrics c2 = scoreSelf(local2, selfq, t_ref, yaw_ref);
        RCLCPP_INFO(
          node_->get_logger(),
          "[%s] DIAG self C2 kf=%d included_kf=%zu excluded_kf=%zu map_vox=%zu query=%zu exact_hit=%.1f%% "
          "yaw_rank=%d/36 yaw_ratio=%.3f fyaw_rank=%d/41 fyaw_argmax=%+ddeg xy_rank=%d/289 xy_argmax=(%.2f,%.2f)",
          name_.c_str(), kf_idx, c2_included, c2_excluded, local2.level(0).voxels.size(), selfq.size(),
          c2.exact_hit_pct, c2.yaw_rank, c2.yaw_ratio, c2.fyaw_rank, c2.fyaw_argmax, c2.xy_rank, c2.xy_argmax_dx,
          c2.xy_argmax_dy);
      }
      local2.releaseMemory();

      // ---------------------------------------------------------------------------------------
      // C3 -- leave-one-out at a SECOND, independent location: repeats C2 for the prior-map
      // keyframe 150 trajectory entries after kf (~300 m along the driven route, wrapping if the
      // trajectory is long enough to make that meaningful). One location scoring badly could be
      // atypical (a corridor, a repetitive stretch); two independent locations agreeing makes the
      // "this scene lacks single-scan observability" conclusion much harder to argue with.
      // ---------------------------------------------------------------------------------------
      // Match by cloud_index rather than gtsam::Key -- buildSelfQuery() returns kf_idx (the
      // keyframe's cloud index into poses6d_), the same value buildPyramid() used to populate
      // each TrajectoryEntry::cloud_index, so this recovers the identical trajectory_ position
      // the old inline key-based lookup did without needing the raw gtsam::Key back out.
      std::size_t kf_traj_idx = trajectory_.size();
      for (std::size_t i = 0; i < trajectory_.size(); ++i) {
        if (trajectory_[i].cloud_index == kf_idx) {
          kf_traj_idx = i;
          break;
        }
      }
      if (trajectory_.empty() || kf_traj_idx >= trajectory_.size()) {
        RCLCPP_INFO(
          node_->get_logger(), "[%s] DIAG self C3: kf not found in trajectory_ (size=%zu), skipping", name_.c_str(),
          trajectory_.size());
      } else {
        const std::size_t idx2 = (kf_traj_idx + 150) % trajectory_.size();
        const TrajectoryEntry & entry2 = trajectory_[idx2];
        if (entry2.cloud_index < 0 || static_cast<std::size_t>(entry2.cloud_index) >= poses6d_->points.size()) {
          RCLCPP_INFO(
            node_->get_logger(), "[%s] DIAG self C3: second keyframe's cloud_index out of range, skipping",
            name_.c_str());
        } else {
          Eigen::Affine3f world_t2 = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(entry2.cloud_index)]);
          Eigen::Isometry3d T_kf2;
          T_kf2.matrix() = world_t2.matrix().cast<double>();
          const gtsam::Vector3 rpy2 = gtsam::Rot3(T_kf2.rotation()).rpy();
          const double roll2 = rpy2(0);
          const double pitch2 = rpy2(1);
          const double yaw2 = rpy2(2);

          std::vector<Eigen::Vector3d> kf2_body;
          retrieveBody(entry2.key, kf2_body);
          const std::vector<Eigen::Vector3d> selfq2 = filterSelfCloud(kf2_body, pitch2, roll2);
          const Eigen::Vector3d t_ref2 = T_kf2.translation();

          std::size_t c3_included = 0, c3_excluded = 0;
          eidos::reloc::VoxelPyramid local3 = buildLocalMap(t_ref2, t_ref2, c3_included, c3_excluded);
          if (selfq2.size() < 20 || local3.empty()) {
            RCLCPP_WARN(
              node_->get_logger(),
              "[%s] DIAG self C3: degenerate (query=%zu, included_kf=%zu, excluded_kf=%zu), skipped", name_.c_str(),
              selfq2.size(), c3_included, c3_excluded);
          } else {
            const SelfMetrics c3 = scoreSelf(local3, selfq2, t_ref2, yaw2);
            RCLCPP_INFO(
              node_->get_logger(),
              "[%s] DIAG self C3 kf=%d included_kf=%zu excluded_kf=%zu map_vox=%zu query=%zu exact_hit=%.1f%% "
              "yaw_rank=%d/36 yaw_ratio=%.3f fyaw_rank=%d/41 fyaw_argmax=%+ddeg xy_rank=%d/289 xy_argmax=(%.2f,%.2f)",
              name_.c_str(), entry2.cloud_index, c3_included, c3_excluded, local3.level(0).voxels.size(),
              selfq2.size(), c3.exact_hit_pct, c3.yaw_rank, c3.yaw_ratio, c3.fyaw_rank, c3.fyaw_argmax, c3.xy_rank,
              c3.xy_argmax_dx, c3.xy_argmax_dy);
          }
          local3.releaseMemory();
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] DIAG self test complete", name_.c_str());
  }

  std::vector<ScoredHypothesis> merged;
  for (auto & v : per_task) {
    for (auto & sh : v) merged.push_back(sh);
  }

  // Merge the fine prefilter's own argmax poses in alongside branch-and-bound's output, so the two
  // compete on equal terms in the sort and NMS below.
  //
  // This is load-bearing, not a belt-and-braces addition. The coarse bound saturates completely on
  // this map -- every root scores the maximum 102000 at the coarsest level (measured: 11611 of
  // 11613 roots tied at the prefilter ceiling) -- so best-first branch-and-bound has no gradient to
  // follow and its expansion order among the tied frontier is arbitrary. The fine prefilter, by
  // contrast, ranks at level 0, which is the one level measured to discriminate (94623-99309 at
  // ground truth versus 77885 for the best wrong pose anywhere on the map). Feeding its argmax
  // poses straight into the candidate list means the correct pose reaches GICP and the acceptance
  // gates on the strength of the discriminative score alone, without depending on the saturated
  // bound to steer the tree search there.
  for (const auto & cand : prefilter_candidates_) merged.push_back(cand);

  eidos::reloc::SearchStats total_stats;
  for (const auto & s : per_task_stats) {
    total_stats.nodes_expanded += s.nodes_expanded;
    total_stats.nodes_pruned += s.nodes_pruned;
    total_stats.point_tests += s.point_tests;
    total_stats.greedy_evaluations += s.greedy_evaluations;
    total_stats.hit_node_cap = total_stats.hit_node_cap || s.hit_node_cap;
  }

  std::sort(merged.begin(), merged.end(), [](const ScoredHypothesis & a, const ScoredHypothesis & b) {
    return a.hyp.score > b.hyp.score;
  });

  for (const auto & cand : merged) {
    bool far_enough = true;
    for (const auto & kept : nms_result) {
      if ((cand.hyp.translation - kept.hyp.translation).norm() <= nms_radius_) {
        far_enough = false;
        break;
      }
    }
    if (far_enough) nms_result.push_back(cand);
  }

  const auto t_end = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] search: score_mode=%s, %zu offsets, %zu tasks, %zu merged hyps -> %zu after NMS, %.1f ms "
    "(nodes_expanded=%zu nodes_pruned=%zu point_tests=%zu hit_node_cap=%d)",
    name_.c_str(),
    scoreModeLabel(active_score_mode_),
    offsets.size(),
    tasks.size(),
    merged.size(),
    nms_result.size(),
    ms,
    total_stats.nodes_expanded,
    total_stats.nodes_pruned,
    total_stats.point_tests,
    total_stats.hit_node_cap ? 1 : 0);
  for (std::size_t i = 0; i < std::min<std::size_t>(3, nms_result.size()); ++i) {
    const auto & h = nms_result[i].hyp;
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s]   #%zu pos=(%.1f,%.1f,%.1f) yaw=%.1f score=%.3f (hit_fraction=%.3f)",
      name_.c_str(),
      i,
      h.translation.x(),
      h.translation.y(),
      h.translation.z(),
      h.yaw * 180.0 / M_PI,
      h.normalized,
      h.hit_fraction);
  }

  // -------------------------------------------------------------------
  // TRACE 5-7: pipeline-order diagnostics against the search's ACTUAL output, run after NMS so
  // nms_result is final. Same debug_probe_pose_ guard as TRACE 1-4 and the PROBE block above;
  // zero cost when unset.
  // -------------------------------------------------------------------
  if (debug_probe_pose_.size() >= 4 && !preps.empty()) {
    const Eigen::Vector3d probe_t(debug_probe_pose_[0], debug_probe_pose_[1], debug_probe_pose_[2]);
    const double probe_yaw = debug_probe_pose_[3] * M_PI / 180.0;
    const auto & pq = preps[0].rotated_query;

    // TRACE 5 -- search output: how close the search's ACTUAL returned hypotheses got to the
    // reference pose, and where the reference pose's own exact leaf score sits relative to the
    // winner's.
    {
      const int ref_leaf_score = eidos::reloc::scorePoseAtLevel(
        pyramid_, pq, probe_t, probe_yaw, 0, hit_weight_, 0, nullptr, active_score_mode_);
      if (nms_result.empty()) {
        RCLCPP_WARN(
          node_->get_logger(), "[%s] TRACE 5 search output: no hypotheses returned; ref_leaf_score=%d",
          name_.c_str(), ref_leaf_score);
      } else {
        const auto & best = nms_result[0].hyp;
        const double best_t_err = (best.translation - probe_t).norm();
        double best_yaw_err_deg = (best.yaw - probe_yaw) * 180.0 / M_PI;
        while (best_yaw_err_deg > 180.0) best_yaw_err_deg -= 360.0;
        while (best_yaw_err_deg <= -180.0) best_yaw_err_deg += 360.0;
        RCLCPP_INFO(
          node_->get_logger(),
          "[%s] TRACE 5 search output: best pos=(%.1f,%.1f,%.1f) yaw=%.1fdeg score=%d t_err=%.2fm "
          "yaw_err=%.1fdeg ref_leaf_score=%d",
          name_.c_str(),
          best.translation.x(),
          best.translation.y(),
          best.translation.z(),
          best.yaw * 180.0 / M_PI,
          best.score,
          best_t_err,
          std::abs(best_yaw_err_deg),
          ref_leaf_score);

        int near_rank = -1;
        for (std::size_t i = 0; i < nms_result.size(); ++i) {
          const auto & h = nms_result[i].hyp;
          const double t_err = (h.translation - probe_t).norm();
          double yaw_err_deg = (h.yaw - probe_yaw) * 180.0 / M_PI;
          while (yaw_err_deg > 180.0) yaw_err_deg -= 360.0;
          while (yaw_err_deg <= -180.0) yaw_err_deg += 360.0;
          if (t_err <= 10.0 && std::abs(yaw_err_deg) <= 20.0) {
            near_rank = static_cast<int>(i);
            RCLCPP_INFO(
              node_->get_logger(),
              "[%s] TRACE 5 search output: hypothesis #%d is within 10m/20deg of reference "
              "(t_err=%.2fm yaw_err=%.1fdeg score=%d)",
              name_.c_str(), near_rank, t_err, std::abs(yaw_err_deg), h.score);
            break;
          }
        }
        if (near_rank < 0) {
          RCLCPP_WARN(
            node_->get_logger(),
            "[%s] TRACE 5 search output: no returned hypothesis within 10m/20deg of reference "
            "(%zu hypotheses checked)",
            name_.c_str(), nms_result.size());
        }
      }
    }

    // TRACE 6 -- BnB versus exhaustive: THE decisive check for implementation correctness.
    // bruteForceCoarse() (declared in bnb_search.hpp) scores every (root, coarsest yaw bin) pair
    // with zero pruning, via the identical scorePoseAtLevel() the search's own frontier-seeding
    // loop calls. If its best coarse cell differs from the coarse cell BnB's own best LEAF pose
    // descended from, that is a pruning/indexing bug in the search; if they agree but neither is
    // near the reference, the search is sound and the SCORE is the problem.
    {
      const eidos::reloc::YawDiscretization trace_yaw_disc =
        eidos::reloc::YawDiscretization::compute(pq, pyramid_);
      if (nms_result.empty() || trace_yaw_disc.max_range <= 0.0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[%s] TRACE 6 BnB vs exhaustive: skipped (no BnB hypotheses or degenerate query)",
          name_.c_str());
      } else {
        const int trace6_coarsest = pyramid_.numLevels() - 1;
        const auto & trace6_lvl = pyramid_.level(trace6_coarsest);
        // active_score_mode_ passed explicitly: bruteForceCoarse()'s trailing ScoreMode
        // parameter defaults to DistanceField, which would silently score against an empty grid
        // if the pyramid fell back to Occupancy.
        const auto brute = eidos::reloc::bruteForceCoarse(pyramid_, pq, roots_, 1, active_score_mode_);

        if (brute.empty()) {
          RCLCPP_WARN(
            node_->get_logger(), "[%s] TRACE 6 BnB vs exhaustive: bruteForceCoarse returned nothing",
            name_.c_str());
        } else {
          const auto & bnb_best = nms_result[0].hyp;
          const auto & brute_best = brute[0];
          const int64_t bnb_cell_ix = eidos::reloc::voxelIndex(bnb_best.translation.x(), trace6_lvl.inv_resolution);
          const int64_t bnb_cell_iy = eidos::reloc::voxelIndex(bnb_best.translation.y(), trace6_lvl.inv_resolution);
          const int64_t bnb_cell_iz = eidos::reloc::voxelIndex(bnb_best.translation.z(), trace6_lvl.inv_resolution);
          const int64_t brute_cell_ix =
            eidos::reloc::voxelIndex(brute_best.translation.x(), trace6_lvl.inv_resolution);
          const int64_t brute_cell_iy =
            eidos::reloc::voxelIndex(brute_best.translation.y(), trace6_lvl.inv_resolution);
          const int64_t brute_cell_iz =
            eidos::reloc::voxelIndex(brute_best.translation.z(), trace6_lvl.inv_resolution);
          const bool coarse_cell_match =
            bnb_cell_ix == brute_cell_ix && bnb_cell_iy == brute_cell_iy && bnb_cell_iz == brute_cell_iz;

          RCLCPP_INFO(
            node_->get_logger(),
            "[%s] TRACE 6 BnB vs exhaustive: brute_best pos=(%.1f,%.1f,%.1f) yaw=%.1fdeg "
            "score=%d(@coarsest) | bnb_best pos=(%.1f,%.1f,%.1f) yaw=%.1fdeg score=%d(@leaf) | "
            "coarse_cell_match=%s",
            name_.c_str(),
            brute_best.translation.x(),
            brute_best.translation.y(),
            brute_best.translation.z(),
            brute_best.yaw * 180.0 / M_PI,
            brute_best.score,
            bnb_best.translation.x(),
            bnb_best.translation.y(),
            bnb_best.translation.z(),
            bnb_best.yaw * 180.0 / M_PI,
            bnb_best.score,
            coarse_cell_match ? "yes" : "NO");

          // Rank of the reference root among ALL roots by brute-force score: reproduces
          // bruteForceCoarse()'s own per-point classification (scorePoseAtLevel() at the
          // coarsest level, its hard-coded hit_weight=3, over the FULL query -- brute force
          // never subsamples, unlike prefilterRoots()) so the rank sits on the same footing as
          // brute_best above, without needing bruteForceCoarse() to expose per-root scores.
          int trace6_root_index = -1;
          const int64_t rix = eidos::reloc::voxelIndex(probe_t.x(), trace6_lvl.inv_resolution);
          const int64_t riy = eidos::reloc::voxelIndex(probe_t.y(), trace6_lvl.inv_resolution);
          const int64_t riz = eidos::reloc::voxelIndex(probe_t.z(), trace6_lvl.inv_resolution);
          for (std::size_t i = 0; i < roots_.size(); ++i) {
            if (roots_[i].ix == rix && roots_[i].iy == riy && roots_[i].iz == riz) {
              trace6_root_index = static_cast<int>(i);
              break;
            }
          }

          if (trace6_root_index < 0) {
            RCLCPP_WARN(
              node_->get_logger(),
              "[%s] TRACE 6 BnB vs exhaustive: reference cell is not a root (see TRACE 2), no rank to report",
              name_.c_str());
          } else {
            const int64_t n_bins6 = trace_yaw_disc.numBins(trace6_coarsest);
            std::vector<int> brute_root_scores(roots_.size(), -1);
#pragma omp parallel for schedule(static) num_threads(num_threads_)
            for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(roots_.size()); ++i) {
              const std::size_t idx = static_cast<std::size_t>(i);
              const auto & root = roots_[idx];
              const Eigen::Vector3d centre =
                eidos::reloc::bnbCellCentre(root.ix, root.iy, root.iz, trace6_lvl.resolution);
              int best = -1;
              for (int64_t k = 0; k < n_bins6; ++k) {
                const double yaw = trace_yaw_disc.binCentre(trace6_coarsest, k);
                const int score = eidos::reloc::scorePoseAtLevel(
                  pyramid_, pq, centre, yaw, trace6_coarsest, 3, 0, nullptr, active_score_mode_);
                if (score > best) best = score;
              }
              brute_root_scores[idx] = best;
            }
            const int ref_brute_score = brute_root_scores[static_cast<std::size_t>(trace6_root_index)];
            int brute_rank = 1;
            for (int s : brute_root_scores) {
              if (s > ref_brute_score) ++brute_rank;
            }
            RCLCPP_INFO(
              node_->get_logger(),
              "[%s] TRACE 6 BnB vs exhaustive: reference root brute-force score=%d rank=%d/%zu",
              name_.c_str(), ref_brute_score, brute_rank, roots_.size());
          }
        }
      }
    }

    // TRACE 7 -- GICP from truth: run the plugin's own GICP polish starting from the REFERENCE
    // pose (same submap-assembly and small_gicp settings gicpPolish() uses). GpsIcpRelocalization
    // gets 69% inliers at this pose; if this plugin's GICP path does not, its GICP setup differs
    // from GpsIcp's, which is a bug independent of anything the search does.
    {
      small_gicp::PointCloud::Ptr trace_live_scan;
      {
        std::lock_guard<std::mutex> lock(scan_lock_);
        trace_live_scan = latest_scan_;
      }
      if (!trace_live_scan || trace_live_scan->empty()) {
        RCLCPP_WARN(
          node_->get_logger(), "[%s] TRACE 7 GICP from truth: no live scan available, skipped", name_.c_str());
      } else {
        auto trace_submap_merged = std::make_shared<small_gicp::PointCloud>();
        for (const auto & entry : trajectory_) {
          if ((entry.position - probe_t).norm() > submap_radius_) continue;
          auto cloud_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(entry.key, pointcloud_from_);
          if (!cloud_opt.has_value() || !*cloud_opt || (*cloud_opt)->empty()) continue;
          Eigen::Affine3f world_t =
            poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(entry.cloud_index)]);
          Eigen::Isometry3d T;
          T.matrix() = world_t.matrix().cast<double>();
          for (const auto & pt : (*cloud_opt)->points) {
            Eigen::Vector3d p = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
            trace_submap_merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
          }
        }
        if (trace_submap_merged->empty()) {
          RCLCPP_WARN(
            node_->get_logger(), "[%s] TRACE 7 GICP from truth: empty submap near reference pose, skipped",
            name_.c_str());
        } else {
          auto [trace_submap, trace_submap_tree] =
            small_gicp::preprocess_points(*trace_submap_merged, submap_leaf_size_, num_neighbors_, num_threads_);

          double trace_imu_roll = 0.0, trace_imu_pitch = 0.0;
          {
            std::lock_guard<std::mutex> lock(imu_lock_);
            trace_imu_roll = latest_imu_roll_;
            trace_imu_pitch = latest_imu_pitch_;
          }
          gtsam::Pose3 trace_init_pose(
            gtsam::Rot3::RzRyRx(trace_imu_roll, trace_imu_pitch, probe_yaw),
            gtsam::Point3(probe_t.x(), probe_t.y(), probe_t.z()));
          Eigen::Isometry3d trace_init_guess;
          trace_init_guess.matrix() = trace_init_pose.matrix();

          small_gicp::RegistrationSetting trace_setting;
          trace_setting.type = small_gicp::RegistrationSetting::GICP;
          trace_setting.max_correspondence_distance = max_correspondence_distance_;
          trace_setting.max_iterations = max_icp_iterations_;
          trace_setting.num_threads = num_threads_;

          auto trace_result = small_gicp::align(
            *trace_submap, *trace_live_scan, *trace_submap_tree, trace_init_guess, trace_setting);
          const double trace_inlier_ratio =
            static_cast<double>(trace_result.num_inliers) / static_cast<double>(trace_live_scan->size());
          const Eigen::Isometry3d trace_delta = trace_init_guess.inverse() * trace_result.T_target_source;
          const auto trace_delta_rpy = gtsam::Rot3(trace_delta.rotation()).rpy();
          // Absolute converged pose (map frame), not just its error from the reference -- lets a
          // caller re-probe (e.g. the debug_res_sweep_ block above) at GICP's own optimum rather
          // than at the GPS-ICP reference pose.
          const auto trace_abs_rpy = gtsam::Rot3(trace_result.T_target_source.rotation()).rpy();
          RCLCPP_INFO(
            node_->get_logger(),
            "[%s] TRACE 7 GICP from truth: converged=%d inliers=%zu (%.0f%%) t_err_from_ref=%.2fm "
            "yaw_err_from_ref=%.1fdeg gicp_pose=(%.2f,%.2f,%.2f) gicp_yaw=%.2fdeg",
            name_.c_str(),
            trace_result.converged ? 1 : 0,
            trace_result.num_inliers,
            trace_inlier_ratio * 100.0,
            trace_delta.translation().norm(),
            std::abs(trace_delta_rpy(2)) * 180.0 / M_PI,
            trace_result.T_target_source.translation().x(),
            trace_result.T_target_source.translation().y(),
            trace_result.T_target_source.translation().z(),
            trace_abs_rpy(2) * 180.0 / M_PI);
        }
      }
    }
  }

  // Estimate the chance floor for this query/map pair, for the uniqueness gate in gicpPolish().
  // See that gate for why a raw score quotient is the wrong comparison.
  //
  // Sampled from ACTUAL corridor root cells at random yaws rather than from a synthetic
  // distribution, so the floor reflects the same kind of pose the runner-up is drawn from: the
  // question the gate asks is "is the winner better than an arbitrary plausible pose on this
  // map", and an arbitrary plausible pose is exactly a random corridor root. Deterministically
  // seeded so a rejected relocalization is reproducible from the log.
  if (!roots_.empty() && !query.empty()) {
    constexpr int kFloorSamples = 96;
    const int coarsest = pyramid_.numLevels() - 1;
    const double res = pyramid_.level(coarsest).resolution;
    std::mt19937 rng(12345);
    std::uniform_int_distribution<std::size_t> pick(0, roots_.size() - 1);
    std::uniform_real_distribution<double> yaw_pick(0.0, 2.0 * M_PI);
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < kFloorSamples; ++i) {
      const auto & r = roots_[pick(rng)];
      const Eigen::Vector3d centre(
        (static_cast<double>(r.ix) + 0.5) * res,
        (static_cast<double>(r.iy) + 0.5) * res,
        (static_cast<double>(r.iz) + 0.5) * res);
      const auto bd = eidos::reloc::scoreBreakdownAtLevel(
        pyramid_, query, centre, yaw_pick(rng), 0, hit_weight_, active_score_mode_);
      sum += static_cast<double>(bd.raw) / static_cast<double>(std::max(1, bd.max_possible));
      ++n;
    }
    last_chance_floor_ = n > 0 ? sum / static_cast<double>(n) : 0.0;
    RCLCPP_INFO(
      node_->get_logger(), "[%s] chance floor: %.3f (mean normalized over %d random corridor poses)",
      name_.c_str(), last_chance_floor_, n);
  } else {
    last_chance_floor_ = 0.0;
  }

  return nms_result;
}

// ---------------------------------------------------------------------------
// Phase D — GICP polish and acceptance
// ---------------------------------------------------------------------------
std::optional<RelocalizationResult> BnbVoxelRelocalization::gicpPolish(
  const std::vector<ScoredHypothesis> & hypotheses)
{
  if (hypotheses.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] no BnB hypotheses survived search", name_.c_str());
    return std::nullopt;
  }

  // Uniqueness gate: a property of the BnB score landscape (is the match ambiguous?), evaluated
  // once against the best spatially distinct runner-up. `hypotheses` is already mutually
  // separated by nms_radius_, so hypotheses[1] (if present) is always that runner-up to
  // hypotheses[0].
  const double best_normalized = hypotheses[0].hyp.normalized;
  const double best_hit_fraction = hypotheses[0].hyp.hit_fraction;
  const double runner_up_normalized = hypotheses.size() > 1 ? hypotheses[1].hyp.normalized : 0.0;
  const double runner_up_hit_fraction = hypotheses.size() > 1 ? hypotheses[1].hyp.hit_fraction : 0.0;
  // The ratio is measured in EXCESS OVER THE CHANCE FLOOR, not as a raw quotient of normalized
  // scores, because neither scoring mode has its "no match at all" value at zero:
  //
  //   - distance_field: a randomly rotated scan still lands most of its points within the kernel
  //     truncation radius of SOME map structure, so it scores a large constant for free. Measured
  //     on ring_road.map: mean normalized 0.563 over 36 random yaws at ground truth.
  //   - occupancy: an all-unknown pose floors at 1/hit_weight (0.33 at the default), never 0.
  //
  // A raw quotient therefore compresses every comparison toward 1.0 and the gate reads a decisive
  // win as ambiguous. Measured, with a query known to be correct: best 0.914, runner-up 0.795 ->
  // raw ratio 1.150, which FAILS the 1.20 default, while the same pair measured above the 0.563
  // floor is 0.351 / 0.232 = 1.51 and passes comfortably. Subtracting the floor makes
  // min_score_ratio mean the same thing in both scoring modes and on maps of differing density,
  // which a raw quotient cannot.
  //
  // `last_chance_floor_` is estimated per search from randomly drawn corridor poses (see
  // estimateChanceFloor()); it is 0.0 when unavailable, which degrades this to the raw quotient.
  const double floor = std::clamp(last_chance_floor_, 0.0, 0.99 * best_normalized);
  const double best_excess = best_normalized - floor;
  const double runner_up_excess = std::max(0.0, runner_up_normalized - floor);
  // A runner-up at or below chance carries no evidence against the winner, so the ratio is
  // unbounded and the gate passes rather than dividing by ~0.
  const bool uniqueness_ok = hypotheses.size() <= 1 || runner_up_excess <= 0.0 ||
    (best_excess >= min_score_ratio_ * runner_up_excess);
  if (!uniqueness_ok) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] rejected: ambiguous match (best=%.3f [hit_fraction=%.3f], runner_up=%.3f [hit_fraction=%.3f], "
      "chance_floor=%.3f, excess %.3f vs %.3f -> ratio %.2f, need >= %.2f)",
      name_.c_str(),
      best_normalized,
      best_hit_fraction,
      runner_up_normalized,
      runner_up_hit_fraction,
      floor,
      best_excess,
      runner_up_excess,
      runner_up_excess > 0.0 ? best_excess / runner_up_excess : 0.0,
      min_score_ratio_);
    return std::nullopt;
  }

  small_gicp::PointCloud::Ptr live_scan;
  {
    std::lock_guard<std::mutex> lock(scan_lock_);
    if (!latest_scan_ || latest_scan_->empty()) {
      RCLCPP_INFO(node_->get_logger(), "[%s] no live scan available for GICP polish", name_.c_str());
      return std::nullopt;
    }
    live_scan = latest_scan_;
  }

  double imu_roll = 0.0, imu_pitch = 0.0;
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    if (has_imu_) {
      imu_roll = latest_imu_roll_;
      imu_pitch = latest_imu_pitch_;
    }
  }

  const int num_candidates = std::min(num_gicp_candidates_, static_cast<int>(hypotheses.size()));
  for (int c = 0; c < num_candidates; ++c) {
    if (stop_requested_.load()) return std::nullopt;

    const auto & sh = hypotheses[static_cast<std::size_t>(c)];
    const auto & hyp = sh.hyp;

    if (hyp.normalized < min_match_score_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] candidate %d rejected: match score %.3f (hit_fraction=%.3f) < %.3f",
        name_.c_str(),
        c,
        hyp.normalized,
        hyp.hit_fraction,
        min_match_score_);
      continue;
    }

    // Assemble world-frame submap: linear scan over the cached trajectory (never getKdTree() --
    // it is non-const and unsafe to call off the SLAM thread).
    auto submap_merged = std::make_shared<small_gicp::PointCloud>();
    for (const auto & entry : trajectory_) {
      if ((entry.position - hyp.translation).norm() > submap_radius_) continue;

      auto cloud_opt = map_manager_->retrieve<pcl::PointCloud<PointType>::Ptr>(entry.key, pointcloud_from_);
      if (!cloud_opt.has_value() || !*cloud_opt || (*cloud_opt)->empty()) continue;

      Eigen::Affine3f world_t = poseTypeToAffine3f(poses6d_->points[static_cast<std::size_t>(entry.cloud_index)]);
      Eigen::Isometry3d T;
      T.matrix() = world_t.matrix().cast<double>();
      for (const auto & pt : (*cloud_opt)->points) {
        Eigen::Vector3d p = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
        submap_merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
      }
    }

    if (submap_merged->empty()) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] candidate %d rejected: empty submap near (%.1f,%.1f,%.1f)",
        name_.c_str(),
        c,
        hyp.translation.x(),
        hyp.translation.y(),
        hyp.translation.z());
      continue;
    }

    auto [submap, submap_tree] =
      small_gicp::preprocess_points(*submap_merged, submap_leaf_size_, num_neighbors_, num_threads_);

    gtsam::Pose3 init_pose(
      gtsam::Rot3::RzRyRx(imu_roll + sh.dr, imu_pitch + sh.dp, hyp.yaw),
      gtsam::Point3(hyp.translation.x(), hyp.translation.y(), hyp.translation.z()));
    Eigen::Isometry3d init_guess;
    init_guess.matrix() = init_pose.matrix();

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = max_icp_iterations_;
    setting.num_threads = num_threads_;

    auto result = small_gicp::align(*submap, *live_scan, *submap_tree, init_guess, setting);

    if (!result.converged) {
      RCLCPP_INFO(node_->get_logger(), "[%s] candidate %d GICP did not converge", name_.c_str(), c);
      continue;
    }

    const double inlier_ratio = static_cast<double>(result.num_inliers) / live_scan->size();
    if (inlier_ratio < min_inlier_ratio_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] candidate %d rejected: inlier ratio %.1f%% < %.1f%%",
        name_.c_str(),
        c,
        inlier_ratio * 100.0,
        min_inlier_ratio_ * 100.0);
      continue;
    }

    // Accepted.
    gtsam::Pose3 relocalized_pose(
      gtsam::Rot3(result.T_target_source.rotation()), gtsam::Point3(result.T_target_source.translation()));
    auto t = relocalized_pose.translation();

    int matched_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < trajectory_.size(); ++i) {
      const double d = (trajectory_[i].position - Eigen::Vector3d(t.x(), t.y(), t.z())).norm();
      if (d < best_dist) {
        best_dist = d;
        matched_index = static_cast<int>(i);
      }
    }

    auto rpy = relocalized_pose.rotation().rpy();
    RCLCPP_INFO(
      node_->get_logger(),
      "\033[32m[%s] RELOCALIZED (BnB): pos=(%.1f,%.1f,%.1f) rpy=(%.1f,%.1f,%.1f) "
      "inliers=%zu (%.0f%%) bnb_score=%.3f (hit_fraction=%.3f)\033[0m",
      name_.c_str(),
      t.x(),
      t.y(),
      t.z(),
      rpy(0) * 180.0 / M_PI,
      rpy(1) * 180.0 / M_PI,
      rpy(2) * 180.0 / M_PI,
      result.num_inliers,
      inlier_ratio * 100.0,
      hyp.normalized,
      hyp.hit_fraction);

    RelocalizationResult reloc_result;
    reloc_result.pose = relocalized_pose;
    reloc_result.fitness_score = result.error;
    reloc_result.matched_keyframe_index = matched_index;
    return reloc_result;
  }

  return std::nullopt;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::BnbVoxelRelocalization, eidos::RelocalizationPlugin)
