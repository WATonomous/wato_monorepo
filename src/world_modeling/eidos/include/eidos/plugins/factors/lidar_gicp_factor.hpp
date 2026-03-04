#pragma once

#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Geometry>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief LiDAR GICP factor plugin (LIO-SAM mapOptimization architecture).
 *
 * All scan matching runs on the SLAM thread in processFrame():
 *   1. Pop latest cloud from queue (filled by lightweight callback)
 *   2. Save Front pose, apply IMU twist increment
 *   3. Preprocess scan, build submap from optimized keyframe poses
 *   4. Scan-to-scan + scan-to-map GICP → update transformTobeMapped_
 *   5. Publish incremental odometry (no hop) and global odometry
 *
 * Threading model (2 mutexes):
 *   - Callback thread: queue clouds (cloud_queue_lock_), buffer IMU twist (imu_odom_lock_)
 *   - SLAM thread: everything else — no cross-thread pose/submap sharing
 */
class LidarGICPFactor : public FactorPlugin {
public:
  LidarGICPFactor() = default;
  ~LidarGICPFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  FactorResult getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;
  bool isReady() const override;
  std::string getReadyStatus() const override;

private:
  // ---- Callbacks (lightweight, callback thread) ----
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---- SLAM thread pipeline methods ----
  void updateInitialGuess(double scan_time);

  std::pair<small_gicp::PointCloud::Ptr,
            std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
      extractSurroundingKeyFrames();

  void scan2MapOptimization(
      const small_gicp::PointCloud::Ptr& source,
      const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>& source_tree,
      const small_gicp::PointCloud::Ptr& target,
      const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>& target_tree);

  void publishOdometry(const builtin_interfaces::msg::Time& stamp);

  // ---- Subscriptions / Publishers ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_global_pub_;

  // ---- Cloud queue (callback → SLAM thread) ----
  struct StampedCloud {
    builtin_interfaces::msg::Time stamp;
    double timestamp;
    small_gicp::PointCloud::Ptr cloud;
  };
  std::deque<StampedCloud> cloud_queue_;
  std::mutex cloud_queue_lock_;

  // ---- IMU odom twist buffer (callback → SLAM thread) ----
  struct ImuOdomSample {
    double timestamp;
    Eigen::Vector3d linear_vel;   // world frame
    Eigen::Vector3d angular_vel;  // body frame
  };
  std::deque<ImuOdomSample> imu_odom_buffer_;
  double last_cloud_time_ = -1;
  std::mutex imu_odom_lock_;

  // ---- LIO-SAM working pose [roll, pitch, yaw, x, y, z] — SLAM thread only ----
  float transformTobeMapped_[6] = {0, 0, 0, 0, 0, 0};

  // ---- Incremental odometry (SLAM thread only) ----
  Eigen::Affine3f incrementalOdometryAffineFront_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f incrementalOdometryAffineBack_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f increOdomAffine_ = Eigen::Affine3f::Identity();
  bool incremental_odom_initialized_ = false;

  // ---- Scan-to-scan state (SLAM thread only) ----
  small_gicp::PointCloud::Ptr prev_scan_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> prev_scan_tree_;
  Eigen::Isometry3d prev_T_ = Eigen::Isometry3d::Identity();

  // ---- Latest scan for MapManager (SLAM thread only, no mutex) ----
  small_gicp::PointCloud::Ptr latest_scan_;

  // ---- GICP quality (SLAM thread only) ----
  Eigen::Matrix<double, 6, 6> last_gicp_H_ = Eigen::Matrix<double, 6, 6>::Zero();
  bool gicp_had_failure_ = false;

  // ---- Local map cache (SLAM thread only) ----
  std::map<int, std::vector<Eigen::Vector4d>> map_cache_;

  // ---- Factor state (SLAM thread only) ----
  gtsam::Pose3 last_keyframe_pose_;
  bool has_last_keyframe_ = false;
  bool has_pose_ = false;

  bool active_ = false;

  // ---- Readiness tracking (atomic — written by callback, read by SLAM thread) ----
  std::atomic<int> clouds_received_{0};

  // ---- Frame names (read-only after init) ----
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";

  // ---- Parameters (read-only after init) ----
  float scan_leaf_size_;
  float submap_leaf_size_;
  int num_threads_;

  int s2s_max_iterations_;
  double s2s_max_correspondence_distance_;

  int s2m_max_iterations_;
  double s2m_max_correspondence_distance_;

  float keyframe_search_radius_;
  float keyframe_density_;
  float recent_keyframe_window_;
  size_t map_cache_max_size_;

  std::vector<double> odom_cov_;
  float min_range_;
  float max_range_;
};

}  // namespace eidos
