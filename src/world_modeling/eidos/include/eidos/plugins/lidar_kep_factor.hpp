#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>

#include "eidos/factor_plugin.hpp"
#include "eidos/scan_matcher.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief LiDAR Keyframe-based Edge-Plane factor plugin.
 *
 * Performs point cloud deskewing, range image projection, LOAM-style
 * feature extraction, scan-to-map matching, and provides BetweenFactor
 * odometry constraints.
 */
class LidarKEPFactor : public FactorPlugin {
public:
  LidarKEPFactor() = default;
  ~LidarKEPFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  std::optional<gtsam::Pose3> processFrame(double timestamp) override;
  std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;

private:
  // ---- Local map assembly (moved from MapManager) ----
  struct LocalMap {
    pcl::PointCloud<PointType>::Ptr corners;
    pcl::PointCloud<PointType>::Ptr surfaces;
  };
  LocalMap buildLocalMap(const PointType& center, float search_radius,
                         float density, float corner_leaf_size,
                         float surf_leaf_size);

  // ---- Callbacks ----
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void imuOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---- Processing pipeline ----
  bool deskewInfo();
  void imuDeskewInfo();
  void projectPointCloud();
  void cloudExtraction();
  void extractFeatures();
  PointType deskewPoint(PointType* point, double rel_time);
  void findRotation(double point_time, float* rot_x, float* rot_y, float* rot_z);

  // ---- IMU extrinsics ----
  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in);

  // ---- Subscriptions ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;

  // ---- Publishers ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_cloud_pub_;

  // ---- Buffers ----
  std::deque<sensor_msgs::msg::PointCloud2> cloud_queue_;
  std::deque<sensor_msgs::msg::Imu> imu_queue_;
  std::mutex imu_lock_;
  std::mutex cloud_lock_;

  // ---- Point clouds ----
  pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmp_ouster_cloud_in_;
  pcl::PointCloud<PointType>::Ptr full_cloud_;
  pcl::PointCloud<PointType>::Ptr extracted_cloud_;
  pcl::PointCloud<PointType>::Ptr corner_cloud_;
  pcl::PointCloud<PointType>::Ptr surface_cloud_;
  pcl::PointCloud<PointType>::Ptr corner_cloud_ds_;
  pcl::PointCloud<PointType>::Ptr surface_cloud_ds_;

  // ---- Voxel filters ----
  pcl::VoxelGrid<PointType> downsample_filter_corner_;
  pcl::VoxelGrid<PointType> downsample_filter_surface_;

  // ---- Scan matcher ----
  std::unique_ptr<ScanMatcher> scan_matcher_;

  // ---- Range image ----
  cv::Mat range_mat_;
  std::vector<int> column_count_vec_;

  // ---- Deskewing buffers ----
  static constexpr int kQueueLength = 2000;
  double imu_time_[kQueueLength];
  double imu_rot_x_[kQueueLength];
  double imu_rot_y_[kQueueLength];
  double imu_rot_z_[kQueueLength];
  int imu_pointer_cur_ = 0;
  bool first_point_flag_ = true;
  Eigen::Affine3f trans_start_inverse_;

  // ---- Cloud info ----
  std::vector<int> start_ring_index_;
  std::vector<int> end_ring_index_;
  std::vector<int> point_col_ind_;
  std::vector<float> point_range_;
  float imu_roll_init_ = 0, imu_pitch_init_ = 0, imu_yaw_init_ = 0;
  bool imu_available_ = false;

  // ---- Feature extraction ----
  struct SmoothnessEntry {
    float value;
    size_t index;
  };
  std::vector<SmoothnessEntry> cloud_smoothness_;
  std::vector<float> cloud_curvature_;
  std::vector<int> cloud_neighbor_picked_;
  std::vector<int> cloud_label_;

  // ---- Scan timing ----
  double time_scan_cur_ = 0;
  double time_scan_end_ = 0;
  std_msgs::msg::Header cloud_header_;

  // ---- IMU odometry initial guess ----
  Eigen::Affine3f imu_odom_affine_front_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f imu_odom_affine_back_ = Eigen::Affine3f::Identity();
  double imu_odom_time_ = -1;
  bool imu_odom_available_ = false;
  std::mutex imu_odom_lock_;

  // ---- Downsampling helper ----
  void downsampleCurrentClouds();

  // ---- Local map cache ----
  mutable std::map<int, std::pair<pcl::PointCloud<PointType>,
                                   pcl::PointCloud<PointType>>> map_cache_;

  // ---- State ----
  float current_transform_[6] = {0};
  gtsam::Pose3 last_keyframe_pose_;
  bool has_last_keyframe_ = false;
  bool new_data_available_ = false;
  bool active_ = false;

  // ---- Parameters ----
  SensorType sensor_type_ = SensorType::VELODYNE;
  int n_scan_ = 32;
  int horizon_scan_ = 1800;
  float min_range_ = 1.0;
  float max_range_ = 100.0;
  float edge_threshold_ = 1.0;
  float surf_threshold_ = 0.1;
  float odom_surf_leaf_size_ = 0.4;
  float mapping_corner_leaf_size_ = 0.2;
  float mapping_surf_leaf_size_ = 0.4;
  int ring_flag_ = 0;
  int deskew_flag_ = 0;
  float keyframe_search_radius_ = 50.0;
  float keyframe_density_ = 2.0;
  float recent_keyframe_window_ = 10.0;  // seconds
  size_t map_cache_max_size_ = 1000;
  float occlusion_depth_diff_ = 0.3;
  float parallel_beam_ratio_ = 0.02;
  double odom_rot_noise_ = 1e-6;
  double odom_trans_noise_ = 1e-4;
  double imu_time_margin_ = 0.01;

  // IMU extrinsics
  Eigen::Matrix3d ext_rot_;
  Eigen::Quaterniond ext_qrpy_;
  Eigen::Vector3d ext_trans_;
};

}  // namespace eidos
