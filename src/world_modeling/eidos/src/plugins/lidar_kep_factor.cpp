#include "eidos/plugins/lidar_kep_factor.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "eidos/slam_core.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void LidarKEPFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".point_cloud_topic", "points_raw");
  node_->declare_parameter(prefix + ".imu_topic", "imu/data");
  node_->declare_parameter(prefix + ".sensor_type", "velodyne");
  node_->declare_parameter(prefix + ".n_scan", 32);
  node_->declare_parameter(prefix + ".horizon_scan", 1800);
  node_->declare_parameter(prefix + ".min_range", 1.0);
  node_->declare_parameter(prefix + ".max_range", 100.0);
  node_->declare_parameter(prefix + ".edge_threshold", 1.0);
  node_->declare_parameter(prefix + ".surf_threshold", 0.1);
  node_->declare_parameter(prefix + ".odom_surf_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".mapping_corner_leaf_size", 0.2);
  node_->declare_parameter(prefix + ".mapping_surf_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".ring_flag", 0);
  node_->declare_parameter(prefix + ".deskew_flag", 0);
  node_->declare_parameter(prefix + ".keyframe_search_radius", 50.0);
  node_->declare_parameter(prefix + ".keyframe_density", 2.0);
  node_->declare_parameter(prefix + ".recent_keyframe_window", 10.0);
  node_->declare_parameter(prefix + ".map_cache_max_size", 1000);
  node_->declare_parameter(prefix + ".occlusion_depth_diff", 0.3);
  node_->declare_parameter(prefix + ".parallel_beam_ratio", 0.02);
  node_->declare_parameter(prefix + ".odom_rot_noise", 1e-6);
  node_->declare_parameter(prefix + ".odom_trans_noise", 1e-4);
  node_->declare_parameter(prefix + ".imu_time_margin", 0.01);

  std::vector<double> identity_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<double> zero_trans = {0, 0, 0};
  node_->declare_parameter(prefix + ".extrinsic_rot", identity_rot);
  node_->declare_parameter(prefix + ".extrinsic_rpy", identity_rot);
  node_->declare_parameter(prefix + ".extrinsic_trans", zero_trans);

  // ---- Read parameters ----
  std::string point_cloud_topic, imu_topic, sensor_type_str;
  node_->get_parameter(prefix + ".point_cloud_topic", point_cloud_topic);
  node_->get_parameter(prefix + ".imu_topic", imu_topic);
  node_->get_parameter(prefix + ".sensor_type", sensor_type_str);
  node_->get_parameter(prefix + ".n_scan", n_scan_);
  node_->get_parameter(prefix + ".horizon_scan", horizon_scan_);
  node_->get_parameter(prefix + ".min_range", min_range_);
  node_->get_parameter(prefix + ".max_range", max_range_);
  node_->get_parameter(prefix + ".edge_threshold", edge_threshold_);
  node_->get_parameter(prefix + ".surf_threshold", surf_threshold_);
  node_->get_parameter(prefix + ".odom_surf_leaf_size", odom_surf_leaf_size_);
  node_->get_parameter(prefix + ".mapping_corner_leaf_size", mapping_corner_leaf_size_);
  node_->get_parameter(prefix + ".mapping_surf_leaf_size", mapping_surf_leaf_size_);
  node_->get_parameter(prefix + ".ring_flag", ring_flag_);
  node_->get_parameter(prefix + ".deskew_flag", deskew_flag_);
  node_->get_parameter(prefix + ".keyframe_search_radius", keyframe_search_radius_);
  node_->get_parameter(prefix + ".keyframe_density", keyframe_density_);
  node_->get_parameter(prefix + ".recent_keyframe_window", recent_keyframe_window_);
  int cache_max = 1000;
  node_->get_parameter(prefix + ".map_cache_max_size", cache_max);
  map_cache_max_size_ = static_cast<size_t>(cache_max);
  node_->get_parameter(prefix + ".occlusion_depth_diff", occlusion_depth_diff_);
  node_->get_parameter(prefix + ".parallel_beam_ratio", parallel_beam_ratio_);
  node_->get_parameter(prefix + ".odom_rot_noise", odom_rot_noise_);
  node_->get_parameter(prefix + ".odom_trans_noise", odom_trans_noise_);
  node_->get_parameter(prefix + ".imu_time_margin", imu_time_margin_);

  if (sensor_type_str == "ouster") sensor_type_ = SensorType::OUSTER;
  else if (sensor_type_str == "livox") sensor_type_ = SensorType::LIVOX;
  else sensor_type_ = SensorType::VELODYNE;

  std::vector<double> ext_rot_v, ext_rpy_v, ext_trans_v;
  node_->get_parameter(prefix + ".extrinsic_rot", ext_rot_v);
  node_->get_parameter(prefix + ".extrinsic_rpy", ext_rpy_v);
  node_->get_parameter(prefix + ".extrinsic_trans", ext_trans_v);

  ext_rot_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
      ext_rot_v.data(), 3, 3);
  Eigen::Matrix3d ext_rpy = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
      ext_rpy_v.data(), 3, 3);
  ext_qrpy_ = Eigen::Quaterniond(ext_rpy);
  ext_trans_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
      ext_trans_v.data(), 3, 1);

  // ---- Allocate point clouds ----
  laser_cloud_in_ = pcl::make_shared<pcl::PointCloud<PointXYZIRT>>();
  tmp_ouster_cloud_in_ = pcl::make_shared<pcl::PointCloud<OusterPointXYZIRT>>();
  full_cloud_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  extracted_cloud_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  corner_cloud_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  surface_cloud_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  corner_cloud_ds_ = pcl::make_shared<pcl::PointCloud<PointType>>();
  surface_cloud_ds_ = pcl::make_shared<pcl::PointCloud<PointType>>();

  full_cloud_->points.resize(n_scan_ * horizon_scan_);
  start_ring_index_.assign(n_scan_, 0);
  end_ring_index_.assign(n_scan_, 0);
  point_col_ind_.assign(n_scan_ * horizon_scan_, 0);
  point_range_.assign(n_scan_ * horizon_scan_, 0);
  column_count_vec_.assign(n_scan_, 0);

  cloud_smoothness_.resize(n_scan_ * horizon_scan_);
  cloud_curvature_.resize(n_scan_ * horizon_scan_, 0);
  cloud_neighbor_picked_.resize(n_scan_ * horizon_scan_, 0);
  cloud_label_.resize(n_scan_ * horizon_scan_, 0);

  // ---- Voxel filters ----
  downsample_filter_corner_.setLeafSize(
      mapping_corner_leaf_size_, mapping_corner_leaf_size_, mapping_corner_leaf_size_);
  downsample_filter_surface_.setLeafSize(
      odom_surf_leaf_size_, odom_surf_leaf_size_, odom_surf_leaf_size_);

  // ---- Create subscriptions ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarKEPFactor::pointCloudCallback, this, std::placeholders::_1),
      sub_opts);
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarKEPFactor::imuCallback, this, std::placeholders::_1),
      sub_opts);

  // ---- Subscribe to IMU odometry for initial guess ----
  // Uses the ImuIntegrationFactor's published odometry topic
  std::string imu_odom_topic;
  node_->declare_parameter(prefix + ".imu_odom_topic",
                           "imu_integration_factor/odometry/imu_incremental");
  node_->get_parameter(prefix + ".imu_odom_topic", imu_odom_topic);

  imu_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      imu_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarKEPFactor::imuOdometryCallback, this,
                std::placeholders::_1),
      sub_opts);

  // ---- Create publishers ----
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      name_ + "/odometry", 10);
  deskewed_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      name_ + "/deskewed_cloud", 10);

  // ---- Scan matcher ----
  int num_cores = 4;
  node_->get_parameter_or("performance.num_cores", num_cores, 4);
  ScanMatcher::Config sm_config;
  sm_config.num_cores = num_cores;
  scan_matcher_ = std::make_unique<ScanMatcher>(sm_config);

  // ---- Init range mat ----
  range_mat_ = cv::Mat(n_scan_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));

  // ---- Register keyframe data types with MapManager ----
  auto& map_manager = core_->getMapManager();
  map_manager.registerType("lidar_kep_factor/corners", {
    // serialize
    [](const std::any& data, const std::string& path) {
      auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
      if (cloud && !cloud->empty()) {
        pcl::io::savePCDFileBinary(path, *cloud);
      }
    },
    // deserialize
    [](const std::string& path) -> std::any {
      auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
      pcl::io::loadPCDFile(path, *cloud);
      return cloud;
    }
  });
  map_manager.registerType("lidar_kep_factor/surfaces", {
    [](const std::any& data, const std::string& path) {
      auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
      if (cloud && !cloud->empty()) {
        pcl::io::savePCDFileBinary(path, *cloud);
      }
    },
    [](const std::string& path) -> std::any {
      auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
      pcl::io::loadPCDFile(path, *cloud);
      return cloud;
    }
  });

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (sensor=%s, %dx%d)",
              name_.c_str(), sensor_type_str.c_str(), n_scan_, horizon_scan_);
}

void LidarKEPFactor::activate() {
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void LidarKEPFactor::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void LidarKEPFactor::reset() {
  {
    std::lock_guard<std::mutex> lock(cloud_lock_);
    cloud_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.clear();
  }
  laser_cloud_in_->clear();
  full_cloud_->clear();
  extracted_cloud_->clear();
  corner_cloud_->clear();
  surface_cloud_->clear();
  corner_cloud_ds_->clear();
  surface_cloud_ds_->clear();
  imu_pointer_cur_ = 0;
  first_point_flag_ = true;
  has_last_keyframe_ = false;
  new_data_available_ = false;
  imu_available_ = false;
  std::fill(std::begin(current_transform_), std::end(current_transform_), 0.0f);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------
void LidarKEPFactor::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cloud_lock_);
  cloud_queue_.push_back(*msg);
}

void LidarKEPFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  sensor_msgs::msg::Imu imu_converted = imuConverter(*msg);
  std::lock_guard<std::mutex> lock(imu_lock_);
  imu_queue_.push_back(imu_converted);
}

void LidarKEPFactor::imuOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imu_odom_lock_);
  Eigen::Affine3f affine = Eigen::Affine3f::Identity();
  affine.translation() = Eigen::Vector3f(
      static_cast<float>(msg->pose.pose.position.x),
      static_cast<float>(msg->pose.pose.position.y),
      static_cast<float>(msg->pose.pose.position.z));
  Eigen::Quaternionf q(
      static_cast<float>(msg->pose.pose.orientation.w),
      static_cast<float>(msg->pose.pose.orientation.x),
      static_cast<float>(msg->pose.pose.orientation.y),
      static_cast<float>(msg->pose.pose.orientation.z));
  affine.linear() = q.toRotationMatrix();

  if (!imu_odom_available_) {
    imu_odom_affine_front_ = affine;
    imu_odom_available_ = true;
  }
  imu_odom_affine_back_ = affine;
  imu_odom_time_ = stamp2Sec(msg->header.stamp);
}

// ---------------------------------------------------------------------------
// processFrame - full LiDAR odometry pipeline
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> LidarKEPFactor::processFrame(double /*timestamp*/) {
  if (!active_) return std::nullopt;

  // Need at least 3 clouds buffered (keep 2 in queue like LIO-SAM)
  sensor_msgs::msg::PointCloud2 current_msg;
  {
    std::lock_guard<std::mutex> lock(cloud_lock_);
    if (cloud_queue_.size() <= 2) return std::nullopt;
    current_msg = std::move(cloud_queue_.front());
    cloud_queue_.pop_front();
  }

  // Reset per-frame state
  laser_cloud_in_->clear();
  extracted_cloud_->clear();
  corner_cloud_->clear();
  surface_cloud_->clear();
  range_mat_ = cv::Mat(n_scan_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));
  imu_pointer_cur_ = 0;
  first_point_flag_ = true;
  imu_available_ = false;
  column_count_vec_.assign(n_scan_, 0);

  // 1. Cache/convert point cloud
  cloud_header_ = current_msg.header;
  if (sensor_type_ == SensorType::VELODYNE || sensor_type_ == SensorType::LIVOX) {
    pcl::moveFromROSMsg(current_msg, *laser_cloud_in_);
  } else if (sensor_type_ == SensorType::OUSTER) {
    pcl::moveFromROSMsg(current_msg, *tmp_ouster_cloud_in_);
    laser_cloud_in_->points.resize(tmp_ouster_cloud_in_->size());
    laser_cloud_in_->is_dense = tmp_ouster_cloud_in_->is_dense;
    for (size_t i = 0; i < tmp_ouster_cloud_in_->size(); i++) {
      auto& src = tmp_ouster_cloud_in_->points[i];
      auto& dst = laser_cloud_in_->points[i];
      dst.x = src.x; dst.y = src.y; dst.z = src.z;
      dst.intensity = src.intensity;
      dst.ring = src.ring;
      dst.time = src.t * 1e-9f;
    }
  }

  time_scan_cur_ = rclcpp::Time(cloud_header_.stamp).seconds();
  if (!laser_cloud_in_->points.empty()) {
    time_scan_end_ = time_scan_cur_ + laser_cloud_in_->points.back().time;
  } else {
    return std::nullopt;
  }

  // Remove NaN
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laser_cloud_in_, *laser_cloud_in_, indices);

  // Check ring channel
  if (ring_flag_ == 0) {
    ring_flag_ = -1;
    for (const auto& field : current_msg.fields) {
      if (field.name == "ring") { ring_flag_ = 1; break; }
    }
    if (ring_flag_ == -1 && sensor_type_ == SensorType::VELODYNE) {
      ring_flag_ = 2;  // Calculate ring from vertical angle
    }
  }

  // Check deskew
  if (deskew_flag_ == 0) {
    deskew_flag_ = -1;
    for (const auto& field : current_msg.fields) {
      if (field.name == "time" || field.name == "t") {
        deskew_flag_ = 1; break;
      }
    }
  }

  // 2. Deskew info (get IMU data for the scan)
  if (!deskewInfo()) return std::nullopt;

  // 3. Project to range image
  projectPointCloud();

  // 4. Extract valid points from range image
  cloudExtraction();

  // 5. Extract features
  extractFeatures();

  // 5b. Apply IMU odometry increment as initial guess update
  {
    std::lock_guard<std::mutex> lock(imu_odom_lock_);
    if (imu_odom_available_) {
      // Compute increment: how much IMU thinks we moved since the last reset
      Eigen::Affine3f imu_increment =
          imu_odom_affine_front_.inverse() * imu_odom_affine_back_;
      // Apply to current transform
      Eigen::Affine3f current_affine = rpyxyzToAffine3f(current_transform_);
      Eigen::Affine3f updated = current_affine * imu_increment;
      float ux, uy, uz, uroll, upitch, uyaw;
      pcl::getTranslationAndEulerAngles(updated, ux, uy, uz, uroll, upitch,
                                        uyaw);
      current_transform_[0] = uroll;
      current_transform_[1] = upitch;
      current_transform_[2] = uyaw;
      current_transform_[3] = ux;
      current_transform_[4] = uy;
      current_transform_[5] = uz;
      // Reset front to back for next increment
      imu_odom_affine_front_ = imu_odom_affine_back_;
    }
  }

  // 6. Scan-to-map matching
  if (core_->getCurrentStateIndex() < 0) {
    // First frame - use IMU orientation as initial guess
    current_transform_[0] = imu_roll_init_;
    current_transform_[1] = imu_pitch_init_;
    current_transform_[2] = imu_yaw_init_;

    new_data_available_ = true;
    auto pose = rpyxyzToGtsamPose3(current_transform_);
    return pose;
  }

  // Build local map for scan matching
  PointType center;
  center.x = current_transform_[3];
  center.y = current_transform_[4];
  center.z = current_transform_[5];

  auto local_map = buildLocalMap(
      center, keyframe_search_radius_, keyframe_density_,
      mapping_corner_leaf_size_, mapping_surf_leaf_size_);

  if (local_map.corners->empty() || local_map.surfaces->empty()) {
    // No local map yet, just return the current transform
    new_data_available_ = true;
    return rpyxyzToGtsamPose3(current_transform_);
  }

  downsampleCurrentClouds();

  // Build KD-trees
  auto corner_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  auto surface_kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  corner_kdtree->setInputCloud(local_map.corners);
  surface_kdtree->setInputCloud(local_map.surfaces);

  // Match
  auto result = scan_matcher_->match(
      corner_cloud_ds_, surface_cloud_ds_,
      local_map.corners, local_map.surfaces,
      corner_kdtree, surface_kdtree,
      current_transform_,
      imu_roll_init_, imu_pitch_init_,
      imu_available_);

  std::copy(result.transform, result.transform + 6, current_transform_);

  // Publish odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = cloud_header_.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "lidar_link";
  odom_msg.pose.pose.position.x = current_transform_[3];
  odom_msg.pose.pose.position.y = current_transform_[4];
  odom_msg.pose.pose.position.z = current_transform_[5];
  tf2::Quaternion q;
  q.setRPY(current_transform_[0], current_transform_[1], current_transform_[2]);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  if (result.degenerate) odom_msg.pose.covariance[0] = 1;
  odom_pub_->publish(odom_msg);

  new_data_available_ = true;
  return rpyxyzToGtsamPose3(current_transform_);
}

// ---------------------------------------------------------------------------
// getFactors - BetweenFactor odometry
// ---------------------------------------------------------------------------
std::vector<gtsam::NonlinearFactor::shared_ptr> LidarKEPFactor::getFactors(
    int state_index, const gtsam::Pose3& state_pose, double /*timestamp*/) {
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  if (!new_data_available_) return factors;
  new_data_available_ = false;

  if (state_index == 0) {
    // First state - no between factor needed (SlamCore adds prior)
    last_keyframe_pose_ = state_pose;
    has_last_keyframe_ = true;
  } else if (has_last_keyframe_) {
    // Add BetweenFactor (odometry constraint)
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << odom_rot_noise_, odom_rot_noise_, odom_rot_noise_,
         odom_trans_noise_, odom_trans_noise_, odom_trans_noise_).finished());
    gtsam::Pose3 relative = last_keyframe_pose_.between(state_pose);
    auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        state_index - 1, state_index, relative, noise);
    factors.push_back(factor);
    last_keyframe_pose_ = state_pose;
    has_last_keyframe_ = true;
  }

  downsampleCurrentClouds();

  // Store keyframe data in MapManager
  auto corners_copy = pcl::make_shared<pcl::PointCloud<PointType>>(*corner_cloud_ds_);
  auto surfaces_copy = pcl::make_shared<pcl::PointCloud<PointType>>(*surface_cloud_ds_);
  auto& map_manager = core_->getMapManager();
  map_manager.addKeyframeData(state_index, "lidar_kep_factor/corners", corners_copy);
  map_manager.addKeyframeData(state_index, "lidar_kep_factor/surfaces", surfaces_copy);

  return factors;
}

void LidarKEPFactor::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool loop_closure_detected) {
  if (loop_closure_detected) {
    map_cache_.clear();
  }
}

// ---------------------------------------------------------------------------
// Local map assembly (moved from MapManager)
// ---------------------------------------------------------------------------
LidarKEPFactor::LocalMap LidarKEPFactor::buildLocalMap(
    const PointType& center, float search_radius,
    float density, float corner_leaf_size, float surf_leaf_size) {
  LocalMap local_map;
  local_map.corners = pcl::make_shared<pcl::PointCloud<PointType>>();
  local_map.surfaces = pcl::make_shared<pcl::PointCloud<PointType>>();

  const auto& map_manager = core_->getMapManager();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();

  if (key_poses_3d->empty()) return local_map;

  // KD-tree to find nearby keyframes
  pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
  kdtree->setInputCloud(key_poses_3d);

  std::vector<int> indices;
  std::vector<float> sq_dists;
  kdtree->radiusSearch(center, search_radius, indices, sq_dists);

  // Downsample key poses for efficiency
  pcl::PointCloud<PointType>::Ptr nearby_poses(
      new pcl::PointCloud<PointType>());
  for (int idx : indices) {
    nearby_poses->push_back(key_poses_3d->points[idx]);
  }

  pcl::VoxelGrid<PointType> pose_filter;
  pose_filter.setLeafSize(density, density, density);
  pcl::PointCloud<PointType>::Ptr nearby_poses_ds(
      new pcl::PointCloud<PointType>());
  pose_filter.setInputCloud(nearby_poses);
  pose_filter.filter(*nearby_poses_ds);

  // Recover original indices after downsampling
  for (auto& pt : nearby_poses_ds->points) {
    kdtree->nearestKSearch(pt, 1, indices, sq_dists);
    pt.intensity = key_poses_3d->points[indices[0]].intensity;
  }

  // Also include recent keyframes (last 10 seconds)
  int num_poses = key_poses_3d->size();
  if (num_poses > 0) {
    double current_time = key_poses_6d->points.back().time;
    for (int i = num_poses - 1; i >= 0; --i) {
      if (current_time - key_poses_6d->points[i].time < recent_keyframe_window_) {
        nearby_poses_ds->push_back(key_poses_3d->points[i]);
      } else {
        break;
      }
    }
  }

  // Assemble corner and surface maps
  pcl::PointCloud<PointType>::Ptr corner_map(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surface_map(
      new pcl::PointCloud<PointType>());

  int num_keyframes = map_manager.numKeyframes();
  for (const auto& pt : nearby_poses_ds->points) {
    if (pointDistance(pt, center) > search_radius) continue;

    int key_idx = static_cast<int>(pt.intensity);
    if (key_idx < 0 || key_idx >= num_keyframes) continue;

    auto it = map_cache_.find(key_idx);
    if (it != map_cache_.end()) {
      *corner_map += it->second.first;
      *surface_map += it->second.second;
    } else {
      auto corner_data = map_manager.getKeyframeData(key_idx, "lidar_kep_factor/corners");
      auto surface_data = map_manager.getKeyframeData(key_idx, "lidar_kep_factor/surfaces");

      if (!corner_data.has_value() || !surface_data.has_value()) continue;

      auto corners = std::any_cast<pcl::PointCloud<PointType>::Ptr>(corner_data.value());
      auto surfaces = std::any_cast<pcl::PointCloud<PointType>::Ptr>(surface_data.value());

      if (!corners || !surfaces) continue;

      PoseType transform = key_poses_6d->points[key_idx];
      auto corner_transformed = transformPointCloud(corners, transform);
      auto surface_transformed = transformPointCloud(surfaces, transform);
      *corner_map += *corner_transformed;
      *surface_map += *surface_transformed;
      map_cache_[key_idx] = std::make_pair(*corner_transformed, *surface_transformed);
    }
  }

  // Downsample the assembled maps
  pcl::VoxelGrid<PointType> corner_filter;
  corner_filter.setLeafSize(corner_leaf_size, corner_leaf_size, corner_leaf_size);
  corner_filter.setInputCloud(corner_map);
  corner_filter.filter(*local_map.corners);

  pcl::VoxelGrid<PointType> surface_filter;
  surface_filter.setLeafSize(surf_leaf_size, surf_leaf_size, surf_leaf_size);
  surface_filter.setInputCloud(surface_map);
  surface_filter.filter(*local_map.surfaces);

  // Clear cache if too large
  if (map_cache_.size() > map_cache_max_size_) {
    map_cache_.clear();
  }

  return local_map;
}

// ---------------------------------------------------------------------------
// Downsample current corner/surface clouds
// ---------------------------------------------------------------------------
void LidarKEPFactor::downsampleCurrentClouds() {
  corner_cloud_ds_->clear();
  downsample_filter_corner_.setInputCloud(corner_cloud_);
  downsample_filter_corner_.filter(*corner_cloud_ds_);

  surface_cloud_ds_->clear();
  pcl::VoxelGrid<PointType> surf_ds;
  surf_ds.setLeafSize(mapping_surf_leaf_size_, mapping_surf_leaf_size_, mapping_surf_leaf_size_);
  surf_ds.setInputCloud(surface_cloud_);
  surf_ds.filter(*surface_cloud_ds_);
}

// ---------------------------------------------------------------------------
// Deskewing
// ---------------------------------------------------------------------------
bool LidarKEPFactor::deskewInfo() {
  std::lock_guard<std::mutex> lock1(imu_lock_);

  if (imu_queue_.empty() ||
      rclcpp::Time(imu_queue_.front().header.stamp).seconds() > time_scan_cur_ ||
      rclcpp::Time(imu_queue_.back().header.stamp).seconds() < time_scan_end_) {
    return false;
  }

  imuDeskewInfo();
  return true;
}

void LidarKEPFactor::imuDeskewInfo() {
  imu_available_ = false;

  while (!imu_queue_.empty()) {
    if (rclcpp::Time(imu_queue_.front().header.stamp).seconds() < time_scan_cur_ - imu_time_margin_)
      imu_queue_.pop_front();
    else
      break;
  }

  if (imu_queue_.empty()) return;

  imu_pointer_cur_ = 0;

  for (size_t i = 0; i < imu_queue_.size(); i++) {
    auto& this_imu = imu_queue_[i];
    double current_imu_time = rclcpp::Time(this_imu.header.stamp).seconds();

    if (current_imu_time <= time_scan_cur_) {
      tf2::Quaternion orientation;
      tf2::fromMsg(this_imu.orientation, orientation);
      double r, p, y;
      tf2::Matrix3x3(orientation).getRPY(r, p, y);
      imu_roll_init_ = static_cast<float>(r);
      imu_pitch_init_ = static_cast<float>(p);
      imu_yaw_init_ = static_cast<float>(y);
    }

    if (current_imu_time > time_scan_end_ + imu_time_margin_) break;

    if (imu_pointer_cur_ == 0) {
      imu_rot_x_[0] = 0; imu_rot_y_[0] = 0; imu_rot_z_[0] = 0;
      imu_time_[0] = current_imu_time;
      ++imu_pointer_cur_;
      continue;
    }

    if (imu_pointer_cur_ >= kQueueLength) break;

    double angular_x = this_imu.angular_velocity.x;
    double angular_y = this_imu.angular_velocity.y;
    double angular_z = this_imu.angular_velocity.z;

    double dt = current_imu_time - imu_time_[imu_pointer_cur_ - 1];
    imu_rot_x_[imu_pointer_cur_] = imu_rot_x_[imu_pointer_cur_ - 1] + angular_x * dt;
    imu_rot_y_[imu_pointer_cur_] = imu_rot_y_[imu_pointer_cur_ - 1] + angular_y * dt;
    imu_rot_z_[imu_pointer_cur_] = imu_rot_z_[imu_pointer_cur_ - 1] + angular_z * dt;
    imu_time_[imu_pointer_cur_] = current_imu_time;
    ++imu_pointer_cur_;
  }

  --imu_pointer_cur_;
  if (imu_pointer_cur_ <= 0) return;

  imu_available_ = true;
}

void LidarKEPFactor::findRotation(
    double point_time, float* rot_x, float* rot_y, float* rot_z) {
  *rot_x = 0; *rot_y = 0; *rot_z = 0;

  int front = 0;
  while (front < imu_pointer_cur_) {
    if (point_time < imu_time_[front]) break;
    ++front;
  }

  if (point_time > imu_time_[front] || front == 0) {
    *rot_x = static_cast<float>(imu_rot_x_[front]);
    *rot_y = static_cast<float>(imu_rot_y_[front]);
    *rot_z = static_cast<float>(imu_rot_z_[front]);
  } else {
    int back = front - 1;
    double ratio_front = (point_time - imu_time_[back]) /
                          (imu_time_[front] - imu_time_[back]);
    double ratio_back = (imu_time_[front] - point_time) /
                         (imu_time_[front] - imu_time_[back]);
    *rot_x = static_cast<float>(
        imu_rot_x_[front] * ratio_front + imu_rot_x_[back] * ratio_back);
    *rot_y = static_cast<float>(
        imu_rot_y_[front] * ratio_front + imu_rot_y_[back] * ratio_back);
    *rot_z = static_cast<float>(
        imu_rot_z_[front] * ratio_front + imu_rot_z_[back] * ratio_back);
  }
}

PointType LidarKEPFactor::deskewPoint(PointType* point, double rel_time) {
  if (deskew_flag_ == -1 || !imu_available_) return *point;

  double point_time = time_scan_cur_ + rel_time;
  float rot_x, rot_y, rot_z;
  findRotation(point_time, &rot_x, &rot_y, &rot_z);

  if (first_point_flag_) {
    trans_start_inverse_ =
        pcl::getTransformation(0, 0, 0, rot_x, rot_y, rot_z).inverse();
    first_point_flag_ = false;
  }

  Eigen::Affine3f trans_final =
      pcl::getTransformation(0, 0, 0, rot_x, rot_y, rot_z);
  Eigen::Affine3f trans_bt = trans_start_inverse_ * trans_final;

  PointType new_point;
  new_point.x = trans_bt(0, 0) * point->x + trans_bt(0, 1) * point->y +
                trans_bt(0, 2) * point->z + trans_bt(0, 3);
  new_point.y = trans_bt(1, 0) * point->x + trans_bt(1, 1) * point->y +
                trans_bt(1, 2) * point->z + trans_bt(1, 3);
  new_point.z = trans_bt(2, 0) * point->x + trans_bt(2, 1) * point->y +
                trans_bt(2, 2) * point->z + trans_bt(2, 3);
  new_point.intensity = point->intensity;
  return new_point;
}

// ---------------------------------------------------------------------------
// Range image projection
// ---------------------------------------------------------------------------
void LidarKEPFactor::projectPointCloud() {
  int cloud_size = static_cast<int>(laser_cloud_in_->points.size());
  for (int i = 0; i < cloud_size; ++i) {
    PointType this_point;
    this_point.x = laser_cloud_in_->points[i].x;
    this_point.y = laser_cloud_in_->points[i].y;
    this_point.z = laser_cloud_in_->points[i].z;
    this_point.intensity = laser_cloud_in_->points[i].intensity;

    float range = pointDistance(this_point);
    if (range < min_range_ || range > max_range_) continue;

    int row = laser_cloud_in_->points[i].ring;
    if (ring_flag_ == 2) {
      float vertical_angle = std::atan2(
          this_point.z,
          std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y)) *
          180.0f / static_cast<float>(M_PI);
      row = static_cast<int>((vertical_angle + (n_scan_ - 1)) / 2.0f);
    }
    if (row < 0 || row >= n_scan_) continue;

    int col = -1;
    if (sensor_type_ == SensorType::VELODYNE || sensor_type_ == SensorType::OUSTER) {
      float horiz_angle = std::atan2(this_point.x, this_point.y) * 180.0f / static_cast<float>(M_PI);
      float ang_res = 360.0f / static_cast<float>(horizon_scan_);
      col = static_cast<int>(-std::round((horiz_angle - 90.0f) / ang_res) + horizon_scan_ / 2);
      if (col >= horizon_scan_) col -= horizon_scan_;
    } else if (sensor_type_ == SensorType::LIVOX) {
      col = column_count_vec_[row];
      column_count_vec_[row]++;
      if (col >= horizon_scan_) continue;
    }
    if (col < 0 || col >= horizon_scan_) continue;
    if (range_mat_.at<float>(row, col) != FLT_MAX) continue;

    this_point = deskewPoint(&this_point, laser_cloud_in_->points[i].time);
    range_mat_.at<float>(row, col) = range;
    int index = col + row * horizon_scan_;
    full_cloud_->points[index] = this_point;
  }
}

// ---------------------------------------------------------------------------
// Cloud extraction from range image
// ---------------------------------------------------------------------------
void LidarKEPFactor::cloudExtraction() {
  int count = 0;
  for (int i = 0; i < n_scan_; ++i) {
    start_ring_index_[i] = count - 1 + 5;
    for (int j = 0; j < horizon_scan_; ++j) {
      if (range_mat_.at<float>(i, j) != FLT_MAX) {
        point_col_ind_[count] = j;
        point_range_[count] = range_mat_.at<float>(i, j);
        extracted_cloud_->push_back(full_cloud_->points[j + i * horizon_scan_]);
        ++count;
      }
    }
    end_ring_index_[i] = count - 1 - 5;
  }
}

// ---------------------------------------------------------------------------
// LOAM-style feature extraction
// ---------------------------------------------------------------------------
void LidarKEPFactor::extractFeatures() {
  corner_cloud_->clear();
  surface_cloud_->clear();

  int cloud_size = static_cast<int>(extracted_cloud_->points.size());

  // Calculate smoothness
  for (int i = 5; i < cloud_size - 5; i++) {
    float diff = point_range_[i - 5] + point_range_[i - 4] +
                 point_range_[i - 3] + point_range_[i - 2] +
                 point_range_[i - 1] - point_range_[i] * 10 +
                 point_range_[i + 1] + point_range_[i + 2] +
                 point_range_[i + 3] + point_range_[i + 4] +
                 point_range_[i + 5];
    cloud_curvature_[i] = diff * diff;
    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = 0;
    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].index = static_cast<size_t>(i);
  }

  // Mark occluded points
  for (int i = 5; i < cloud_size - 6; ++i) {
    float depth1 = point_range_[i];
    float depth2 = point_range_[i + 1];
    int col_diff = std::abs(point_col_ind_[i + 1] - point_col_ind_[i]);
    if (col_diff < 10) {
      if (depth1 - depth2 > occlusion_depth_diff_) {
        for (int l = 0; l <= 5; l++) cloud_neighbor_picked_[i - l] = 1;
      } else if (depth2 - depth1 > occlusion_depth_diff_) {
        for (int l = 1; l <= 6; l++) cloud_neighbor_picked_[i + l] = 1;
      }
    }
    float diff1 = std::abs(point_range_[i - 1] - point_range_[i]);
    float diff2 = std::abs(point_range_[i + 1] - point_range_[i]);
    if (diff1 > parallel_beam_ratio_ * point_range_[i] && diff2 > parallel_beam_ratio_ * point_range_[i])
      cloud_neighbor_picked_[i] = 1;
  }

  // Extract features per scan ring
  auto surface_scan = pcl::make_shared<pcl::PointCloud<PointType>>();
  auto surface_scan_ds = pcl::make_shared<pcl::PointCloud<PointType>>();
  pcl::VoxelGrid<PointType> surf_downsample;
  surf_downsample.setLeafSize(odom_surf_leaf_size_, odom_surf_leaf_size_, odom_surf_leaf_size_);

  for (int i = 0; i < n_scan_; i++) {
    surface_scan->clear();

    for (int j = 0; j < 6; j++) {
      int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
      int ep = (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 - 1;
      if (sp >= ep) continue;

      std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
                [](const SmoothnessEntry& a, const SmoothnessEntry& b) {
                  return a.value < b.value;
                });

      // Pick corner features (high curvature)
      int largest_picked = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = static_cast<int>(cloud_smoothness_[k].index);
        if (cloud_neighbor_picked_[ind] == 0 &&
            cloud_curvature_[ind] > edge_threshold_) {
          largest_picked++;
          if (largest_picked <= 20) {
            cloud_label_[ind] = 1;
            corner_cloud_->push_back(extracted_cloud_->points[ind]);
          } else {
            break;
          }
          cloud_neighbor_picked_[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if (ind + l >= cloud_size) break;
            int col_diff = std::abs(point_col_ind_[ind + l] - point_col_ind_[ind + l - 1]);
            if (col_diff > 10) break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (ind + l < 0) break;
            int col_diff = std::abs(point_col_ind_[ind + l] - point_col_ind_[ind + l + 1]);
            if (col_diff > 10) break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      // Pick surface features (low curvature)
      for (int k = sp; k <= ep; k++) {
        int ind = static_cast<int>(cloud_smoothness_[k].index);
        if (cloud_neighbor_picked_[ind] == 0 &&
            cloud_curvature_[ind] < surf_threshold_) {
          cloud_label_[ind] = -1;
          cloud_neighbor_picked_[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if (ind + l >= cloud_size) break;
            int col_diff = std::abs(point_col_ind_[ind + l] - point_col_ind_[ind + l - 1]);
            if (col_diff > 10) break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (ind + l < 0) break;
            int col_diff = std::abs(point_col_ind_[ind + l] - point_col_ind_[ind + l + 1]);
            if (col_diff > 10) break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloud_label_[k] <= 0) {
          surface_scan->push_back(extracted_cloud_->points[k]);
        }
      }
    }

    surface_scan_ds->clear();
    surf_downsample.setInputCloud(surface_scan);
    surf_downsample.filter(*surface_scan_ds);
    *surface_cloud_ += *surface_scan_ds;
  }
}

// ---------------------------------------------------------------------------
// IMU extrinsic conversion
// ---------------------------------------------------------------------------
sensor_msgs::msg::Imu LidarKEPFactor::imuConverter(
    const sensor_msgs::msg::Imu& imu_in) {
  sensor_msgs::msg::Imu imu_out = imu_in;

  Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                      imu_in.linear_acceleration.y,
                      imu_in.linear_acceleration.z);
  acc = ext_rot_ * acc;
  imu_out.linear_acceleration.x = acc.x();
  imu_out.linear_acceleration.y = acc.y();
  imu_out.linear_acceleration.z = acc.z();

  Eigen::Vector3d gyr(imu_in.angular_velocity.x,
                      imu_in.angular_velocity.y,
                      imu_in.angular_velocity.z);
  gyr = ext_rot_ * gyr;
  imu_out.angular_velocity.x = gyr.x();
  imu_out.angular_velocity.y = gyr.y();
  imu_out.angular_velocity.z = gyr.z();

  Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
                            imu_in.orientation.y, imu_in.orientation.z);
  Eigen::Quaterniond q_final = q_from * ext_qrpy_;
  imu_out.orientation.x = q_final.x();
  imu_out.orientation.y = q_final.y();
  imu_out.orientation.z = q_final.z();
  imu_out.orientation.w = q_final.w();

  return imu_out;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LidarKEPFactor, eidos::FactorPlugin)
