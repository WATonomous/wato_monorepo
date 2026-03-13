#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>

#include <pcl/point_cloud.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/registration/registration_result.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

class LisoFactor : public FactorPlugin {
public:
  LisoFactor() = default;
  ~LisoFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  StampedFactorResult getFactors(gtsam::Key key) override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

  bool isReady() const override;
  std::string getReadyStatus() const override;
  bool hasData() const override;

  std::pair<std::string, std::string> getTfFrames() const override {
    return {odom_frame_, base_link_frame_};
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void rebuildSubmap();
  std::vector<gtsam::Key> bfsCollectStates(gtsam::Key start, double radius);

  // State
  std::atomic<bool> first_scan_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> scan_received_{false};

  // Async submap rebuild guard
  std::atomic<bool> rebuild_in_progress_{false};

  // Cached submap (read by lidarCallback, written by rebuildSubmap)
  small_gicp::PointCloud::Ptr cached_submap_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> cached_submap_tree_;
  mutable std::shared_mutex submap_mtx_;

  // Cached GICP result for getFactors
  small_gicp::RegistrationResult cached_result_;
  gtsam::Pose3 cached_pose_;
  small_gicp::PointCloud::Ptr cached_cloud_;  // body-frame (for submap assembly)
  pcl::PointCloud<PointType>::Ptr cached_pcl_cloud_;  // body-frame (for MapManager/visualizers)
  double cached_timestamp_ = 0.0;
  bool has_cached_result_ = false;
  mutable std::mutex result_mtx_;

  // Subscription + publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;              // map frame (absolute)
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_incremental_pub_;  // odom frame (incremental)

  // Last successful match tracking (for initial guess computation)
  gtsam::Pose3 last_matched_pose_;       // GICP result at last successful match
  bool has_last_match_ = false;

  // IMU buffer (independent of motion model)
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::mutex imu_mtx_;

  // Gyro-integrated rotation tracking for initial guess
  Eigen::Vector3d gyro_delta_rpy_{0, 0, 0};  // accumulated rotation since last match
  double last_gyro_time_ = 0.0;               // timestamp of last integrated sample
  bool gyro_tracking_active_ = false;

  // TF: base_link <- imu_link (for transforming angular velocity)
  Eigen::Matrix3d R_base_imu_ = Eigen::Matrix3d::Identity();
  bool has_imu_tf_ = false;

  // IMU warmup: stationarity detection + gravity alignment (independent of motion model)
  std::atomic<bool> imu_warmup_complete_{false};
  int imu_warmup_count_ = 0;               // consecutive stationary samples
  Eigen::Vector4d warmup_quat_sum_{0, 0, 0, 0};  // accumulated quaternion (wxyz) for averaging
  bool warmup_quat_hemisphere_set_ = false;
  Eigen::Quaterniond warmup_quat_reference_;       // first quaternion, for hemisphere consistency
  gtsam::Rot3 initial_gravity_orientation_; // computed during warmup

  // Incremental odometry tracking (odom frame, never corrected)
  gtsam::Pose3 incremental_pose_;           // running pose in odom frame
  gtsam::Pose3 prev_incremental_pose_;      // last GICP result (for computing delta)
  bool has_prev_incremental_ = false;
  double prev_odom_time_ = 0.0;             // timestamp of previous match (for twist dt)

  // Distance gating + relative factor tracking
  gtsam::Point3 last_factor_position_ = gtsam::Point3(0, 0, 0);
  bool has_last_factor_ = false;
  gtsam::Key prev_liso_key_ = 0;
  gtsam::Pose3 prev_liso_pose_;
  bool has_prev_liso_ = false;

  // Config
  double scan_ds_resolution_ = 0.5;
  double submap_ds_resolution_ = 0.5;
  double submap_radius_ = 50.0;
  double max_correspondence_distance_ = 2.0;
  int max_iterations_ = 20;
  int num_threads_ = 4;
  int num_neighbors_ = 10;
  int max_submap_states_ = 25;
  int min_inliers_ = 50;
  double min_noise_ = 0.01;
  double min_scan_distance_ = 1.0;
  std::string lidar_frame_ = "velodyne";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_footprint";
  std::string imu_topic_ = "/imu/data";
  std::string imu_frame_ = "imu_link";
  int imu_warmup_samples_ = 200;
  double imu_stationary_gyr_threshold_ = 0.005;  // rad/s
  std::vector<double> odom_pose_cov_ = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
  std::vector<double> odom_twist_cov_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_tf_ = false;
};

}  // namespace eidos
