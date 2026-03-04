#include "eidos/plugins/factors/lidar_gicp_factor.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <gtsam/slam/BetweenFactor.h>

#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/slam_core.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void LidarGICPFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".point_cloud_topic", "points_raw");
  node_->declare_parameter(prefix + ".imu_odom_topic",
                           "imu_integration_factor/odometry/imu_incremental");
  node_->declare_parameter(prefix + ".odom_topic", name_ + "/odometry");
  node_->declare_parameter(prefix + ".odom_global_topic",
                           name_ + "/odometry/global");
  node_->declare_parameter(prefix + ".min_range", 1.0);
  node_->declare_parameter(prefix + ".max_range", 100.0);
  node_->declare_parameter(prefix + ".scan_leaf_size", 0.5);
  node_->declare_parameter(prefix + ".submap_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".num_threads", 4);
  node_->declare_parameter(prefix + ".s2s_max_iterations", 10);
  node_->declare_parameter(prefix + ".s2s_max_correspondence_distance", 1.0);
  node_->declare_parameter(prefix + ".s2m_max_iterations", 30);
  node_->declare_parameter(prefix + ".s2m_max_correspondence_distance", 2.0);
  node_->declare_parameter(prefix + ".keyframe_search_radius", 50.0);
  node_->declare_parameter(prefix + ".keyframe_density", 2.0);
  node_->declare_parameter(prefix + ".recent_keyframe_window", 10.0);
  node_->declare_parameter(prefix + ".map_cache_max_size", 1000);
  node_->declare_parameter(prefix + ".odom_cov",
      std::vector<double>{1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4});

  // ---- Read parameters ----
  std::string point_cloud_topic, imu_odom_topic, odom_topic, odom_global_topic;
  node_->get_parameter(prefix + ".point_cloud_topic", point_cloud_topic);
  node_->get_parameter(prefix + ".imu_odom_topic", imu_odom_topic);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".odom_global_topic", odom_global_topic);

  double min_range_d, max_range_d;
  node_->get_parameter(prefix + ".min_range", min_range_d);
  node_->get_parameter(prefix + ".max_range", max_range_d);
  min_range_ = static_cast<float>(min_range_d);
  max_range_ = static_cast<float>(max_range_d);

  double scan_leaf, submap_leaf;
  node_->get_parameter(prefix + ".scan_leaf_size", scan_leaf);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf);
  scan_leaf_size_ = static_cast<float>(scan_leaf);
  submap_leaf_size_ = static_cast<float>(submap_leaf);

  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".s2s_max_iterations", s2s_max_iterations_);
  node_->get_parameter(prefix + ".s2s_max_correspondence_distance",
                       s2s_max_correspondence_distance_);
  node_->get_parameter(prefix + ".s2m_max_iterations", s2m_max_iterations_);
  node_->get_parameter(prefix + ".s2m_max_correspondence_distance",
                       s2m_max_correspondence_distance_);

  double kf_radius, kf_density, kf_window;
  node_->get_parameter(prefix + ".keyframe_search_radius", kf_radius);
  node_->get_parameter(prefix + ".keyframe_density", kf_density);
  node_->get_parameter(prefix + ".recent_keyframe_window", kf_window);
  keyframe_search_radius_ = static_cast<float>(kf_radius);
  keyframe_density_ = static_cast<float>(kf_density);
  recent_keyframe_window_ = static_cast<float>(kf_window);

  int cache_max = 1000;
  node_->get_parameter(prefix + ".map_cache_max_size", cache_max);
  map_cache_max_size_ = static_cast<size_t>(cache_max);

  node_->get_parameter(prefix + ".odom_cov", odom_cov_);

  // ---- Frame names ----
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  // ---- Create subscriptions ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarGICPFactor::pointCloudCallback, this,
                std::placeholders::_1),
      sub_opts);

  imu_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      imu_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarGICPFactor::imuOdometryCallback, this,
                std::placeholders::_1),
      sub_opts);

  // ---- Create publishers ----
  odom_incremental_pub_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  odom_global_pub_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(odom_global_topic, 10);

  // ---- Register keyframe data type with MapManager ----
  auto& map_manager = core_->getMapManager();
  map_manager.registerType("lidar_gicp_factor/cloud", {
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

  RCLCPP_INFO(node_->get_logger(),
              "[%s] initialized (scan_leaf=%.2f, submap_leaf=%.2f, "
              "s2s: iter=%d corr=%.1f, s2m: iter=%d corr=%.1f, threads=%d)",
              name_.c_str(), scan_leaf_size_, submap_leaf_size_,
              s2s_max_iterations_, s2s_max_correspondence_distance_,
              s2m_max_iterations_, s2m_max_correspondence_distance_,
              num_threads_);
}

void LidarGICPFactor::activate() {
  active_ = true;
  odom_incremental_pub_->on_activate();
  odom_global_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void LidarGICPFactor::deactivate() {
  active_ = false;
  odom_incremental_pub_->on_deactivate();
  odom_global_pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void LidarGICPFactor::reset() {
  {
    std::lock_guard<std::mutex> lock(cloud_queue_lock_);
    cloud_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(imu_odom_lock_);
    imu_odom_buffer_.clear();
    last_cloud_time_ = -1;
  }
  latest_scan_.reset();
  prev_scan_.reset();
  prev_scan_tree_.reset();
  prev_T_ = Eigen::Isometry3d::Identity();
  std::fill(std::begin(transformTobeMapped_), std::end(transformTobeMapped_), 0.0f);
  incrementalOdometryAffineFront_ = Eigen::Affine3f::Identity();
  incrementalOdometryAffineBack_ = Eigen::Affine3f::Identity();
  increOdomAffine_ = Eigen::Affine3f::Identity();
  incremental_odom_initialized_ = false;
  has_pose_ = false;
  gicp_had_failure_ = false;
  last_gicp_H_.setZero();
  has_last_keyframe_ = false;
  clouds_received_.store(0, std::memory_order_relaxed);
  map_cache_.clear();
}

// ---------------------------------------------------------------------------
// isReady
// ---------------------------------------------------------------------------
bool LidarGICPFactor::isReady() const {
  return clouds_received_.load(std::memory_order_relaxed) >= 3;
}

std::string LidarGICPFactor::getReadyStatus() const {
  int count = clouds_received_.load(std::memory_order_relaxed);
  return "clouds received: " + std::to_string(count) + "/3";
}

// ---------------------------------------------------------------------------
// pointCloudCallback — lightweight: convert, range-filter, queue
// ---------------------------------------------------------------------------
void LidarGICPFactor::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!active_) return;

  int count = clouds_received_.fetch_add(1, std::memory_order_relaxed);
  if (count == 0) {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] first point cloud received (stamp: %.3f)",
                name_.c_str(), rclcpp::Time(msg->header.stamp).seconds());
  }

  // Convert PointCloud2 → small_gicp::PointCloud
  auto scan = fromRosMsg(*msg);
  if (!scan || scan->empty()) return;

  // Range filter
  {
    auto& pts = scan->points;
    size_t j = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      double r = pts[i].head<3>().norm();
      if (r >= static_cast<double>(min_range_) &&
          r <= static_cast<double>(max_range_)) {
        pts[j++] = pts[i];
      }
    }
    pts.resize(j);
  }
  if (scan->empty()) return;

  // Push to queue (cap at 5)
  {
    std::lock_guard<std::mutex> lock(cloud_queue_lock_);
    cloud_queue_.push_back({msg->header.stamp,
                            rclcpp::Time(msg->header.stamp).seconds(),
                            scan});
    while (cloud_queue_.size() > 5) {
      cloud_queue_.pop_front();
    }
  }
}

// ---------------------------------------------------------------------------
// imuOdometryCallback — buffer twist samples
// ---------------------------------------------------------------------------
void LidarGICPFactor::imuOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imu_odom_lock_);
  ImuOdomSample sample;
  sample.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.linear_vel = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
  sample.angular_vel = Eigen::Vector3d(
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);
  imu_odom_buffer_.push_back(sample);
}

// ---------------------------------------------------------------------------
// processFrame — main workhorse (SLAM thread)
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> LidarGICPFactor::processFrame(double /*timestamp*/) {
  if (!active_) return std::nullopt;

  // Not tracking yet — nothing to do
  if (core_->getState() != SlamState::TRACKING) return std::nullopt;

  // Pop latest cloud from queue
  StampedCloud latest;
  {
    std::lock_guard<std::mutex> lock(cloud_queue_lock_);
    if (cloud_queue_.empty()) {
      if (has_pose_) return rpyxyzToGtsamPose3(transformTobeMapped_);
      return std::nullopt;
    }
    latest = cloud_queue_.back();
    cloud_queue_.clear();
  }

  // 1. Save Front = pose BEFORE matching (LIO-SAM line 790)
  incrementalOdometryAffineFront_ = rpyxyzToAffine3f(transformTobeMapped_);

  // 2. Apply IMU twist increment
  updateInitialGuess(latest.timestamp);

  RCLCPP_INFO(node_->get_logger(),
      "[%s] processFrame: after IMU update, pose=[%.3f, %.3f, %.3f]",
      name_.c_str(),
      transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]);

  // 3. Preprocess scan (downsample + normals/covariances + KD-tree)
  auto [source, source_tree] = small_gicp::preprocess_points(
      *latest.cloud, static_cast<double>(scan_leaf_size_), 10, num_threads_);

  RCLCPP_INFO(node_->get_logger(),
      "[%s] scan preprocessed: %zu pts (leaf=%.2f)",
      name_.c_str(), source->size(), scan_leaf_size_);

  // 4. Build submap from optimized keyframe poses
  auto [target, target_tree] = extractSurroundingKeyFrames();

  // 5. Scan-to-scan + scan-to-map GICP
  scan2MapOptimization(source, source_tree, target, target_tree);

  // 6. Store latest raw scan for MapManager
  latest_scan_ = latest.cloud;

  // 7. Update prev scan/pose for next S2S
  prev_scan_ = source;
  prev_scan_tree_ = source_tree;
  // Build working Isometry3d from transformTobeMapped_ for prev_T_
  prev_T_ = Eigen::Isometry3d::Identity();
  Eigen::Affine3f working = rpyxyzToAffine3f(transformTobeMapped_);
  prev_T_.linear() = working.rotation().cast<double>();
  prev_T_.translation() = working.translation().cast<double>();

  has_pose_ = true;

  // 8. Publish odometry
  publishOdometry(latest.stamp);

  // 9. Return pose to SLAM core
  return rpyxyzToGtsamPose3(transformTobeMapped_);
}

// ---------------------------------------------------------------------------
// updateInitialGuess — integrate IMU twist into transformTobeMapped_
// ---------------------------------------------------------------------------
void LidarGICPFactor::updateInitialGuess(double scan_time) {
  // Build current working pose as Affine3f
  Eigen::Affine3f working = rpyxyzToAffine3f(transformTobeMapped_);

  int imu_samples_used = 0;
  {
    std::lock_guard<std::mutex> lock(imu_odom_lock_);
    double prev_t = last_cloud_time_;

    while (!imu_odom_buffer_.empty()) {
      const auto& sample = imu_odom_buffer_.front();
      if (sample.timestamp > scan_time) break;

      if (prev_t >= 0) {
        double dt = sample.timestamp - prev_t;
        if (dt > 0 && dt < 1.0) {
          // Apply angular velocity (body frame) as rotation increment
          double angle = sample.angular_vel.norm() * dt;
          if (angle > 1e-10) {
            Eigen::Matrix3f rot_incr =
                Eigen::AngleAxisd(angle, sample.angular_vel.normalized())
                    .toRotationMatrix().cast<float>();
            working.linear() = working.linear() * rot_incr;
          }
          // Apply linear velocity (world frame) as translation increment
          Eigen::Vector3f trans_step = sample.linear_vel.cast<float>() *
                                      static_cast<float>(dt);
          working.translation() += trans_step;
          ++imu_samples_used;
        }
      }
      prev_t = sample.timestamp;
      imu_odom_buffer_.pop_front();
    }

    last_cloud_time_ = scan_time;
  }

  // Write back to transformTobeMapped_ [roll, pitch, yaw, x, y, z]
  // pcl::getTranslationAndEulerAngles returns (x, y, z, roll, pitch, yaw)
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(working, x, y, z, roll, pitch, yaw);
  transformTobeMapped_[0] = roll;
  transformTobeMapped_[1] = pitch;
  transformTobeMapped_[2] = yaw;
  transformTobeMapped_[3] = x;
  transformTobeMapped_[4] = y;
  transformTobeMapped_[5] = z;

  RCLCPP_DEBUG(node_->get_logger(),
      "[%s] updateInitialGuess: %d IMU samples consumed", name_.c_str(),
      imu_samples_used);
}

// ---------------------------------------------------------------------------
// extractSurroundingKeyFrames — build submap from optimized keyframe poses
// ---------------------------------------------------------------------------
std::pair<small_gicp::PointCloud::Ptr,
          std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>>
LidarGICPFactor::extractSurroundingKeyFrames() {
  const auto& map_manager = core_->getMapManager();
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_keyframes = map_manager.numKeyframes();

  if (num_keyframes == 0) {
    RCLCPP_WARN(node_->get_logger(), "[%s] extractSurroundingKeyFrames: 0 keyframes",
                name_.c_str());
    return {nullptr, nullptr};
  }

  // Current position as center for radius search
  Eigen::Vector3d center(transformTobeMapped_[3], transformTobeMapped_[4],
                         transformTobeMapped_[5]);

  // Select nearby keyframes with spatial thinning via voxel hash
  std::set<std::tuple<int, int, int>> occupied;
  std::vector<int> selected;

  double latest_time = key_poses_6d->points[num_keyframes - 1].time;

  for (int i = 0; i < num_keyframes; ++i) {
    const auto& p3 = key_poses_3d->points[i];
    Eigen::Vector3d pos(p3.x, p3.y, p3.z);

    double dist = (pos - center).norm();
    if (dist > static_cast<double>(keyframe_search_radius_)) continue;

    bool recent = (latest_time - key_poses_6d->points[i].time) <
                  static_cast<double>(recent_keyframe_window_);
    if (recent) {
      selected.push_back(i);
      continue;
    }

    auto key = std::make_tuple(
        static_cast<int>(std::floor(pos.x() / keyframe_density_)),
        static_cast<int>(std::floor(pos.y() / keyframe_density_)),
        static_cast<int>(std::floor(pos.z() / keyframe_density_)));
    if (occupied.insert(key).second) {
      selected.push_back(i);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
      "[%s] extractSurroundingKeyFrames: %d keyframes total, %zu selected (cache=%zu)",
      name_.c_str(), num_keyframes, selected.size(), map_cache_.size());

  if (selected.empty()) return {nullptr, nullptr};

  // Assemble submap using map_cache_
  auto submap = std::make_shared<small_gicp::PointCloud>();
  int cache_hits = 0, loaded = 0, missing = 0;

  for (int idx : selected) {
    auto it = map_cache_.find(idx);
    if (it != map_cache_.end()) {
      submap->points.insert(submap->points.end(),
                            it->second.begin(), it->second.end());
      ++cache_hits;
      continue;
    }

    auto cloud_data = map_manager.getKeyframeData(idx, "lidar_gicp_factor/cloud");
    if (!cloud_data.has_value()) {
      ++missing;
      continue;
    }

    auto pcl_cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(
        cloud_data.value());
    if (!pcl_cloud || pcl_cloud->empty()) {
      ++missing;
      continue;
    }

    const auto& pt = key_poses_6d->points[idx];
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() =
        (Eigen::AngleAxisd(static_cast<double>(pt.yaw),
                           Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(static_cast<double>(pt.pitch),
                           Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(static_cast<double>(pt.roll),
                           Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    T.translation() = Eigen::Vector3d(pt.x, pt.y, pt.z);

    std::vector<Eigen::Vector4d> transformed(pcl_cloud->size());
    for (size_t i = 0; i < pcl_cloud->size(); ++i) {
      const auto& p = pcl_cloud->points[i];
      Eigen::Vector3d pw = T * Eigen::Vector3d(p.x, p.y, p.z);
      transformed[i] = Eigen::Vector4d(pw.x(), pw.y(), pw.z(), 1.0);
    }

    submap->points.insert(submap->points.end(),
                          transformed.begin(), transformed.end());
    map_cache_[idx] = std::move(transformed);
    ++loaded;
  }

  RCLCPP_INFO(node_->get_logger(),
      "[%s] extractSurroundingKeyFrames: assembled %zu pts "
      "(cache_hits=%d, loaded=%d, missing=%d)",
      name_.c_str(), submap->points.size(), cache_hits, loaded, missing);

  if (map_cache_.size() > map_cache_max_size_) {
    RCLCPP_INFO(node_->get_logger(),
        "[%s] map cache exceeded max size (%zu > %zu) — clearing",
        name_.c_str(), map_cache_.size(), map_cache_max_size_);
    map_cache_.clear();
  }

  if (submap->empty()) return {nullptr, nullptr};

  // Preprocess submap (downsample + normals + KD-tree)
  auto [target, tree] = small_gicp::preprocess_points(
      *submap, static_cast<double>(submap_leaf_size_), 10, num_threads_);

  RCLCPP_INFO(node_->get_logger(),
      "[%s] submap preprocessed: raw=%zu → %zu pts (leaf=%.2f)",
      name_.c_str(), submap->size(), target->size(), submap_leaf_size_);

  return {target, tree};
}

// ---------------------------------------------------------------------------
// scan2MapOptimization — S2S + S2M GICP, updates transformTobeMapped_
// ---------------------------------------------------------------------------
void LidarGICPFactor::scan2MapOptimization(
    const small_gicp::PointCloud::Ptr& source,
    const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>& source_tree,
    const small_gicp::PointCloud::Ptr& target,
    const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>>& target_tree) {

  // Build working Isometry3d from transformTobeMapped_
  Eigen::Affine3f working_af = rpyxyzToAffine3f(transformTobeMapped_);
  Eigen::Isometry3d working_T = Eigen::Isometry3d::Identity();
  working_T.linear() = working_af.rotation().cast<double>();
  working_T.translation() = working_af.translation().cast<double>();

  // ---- Step 1: Scan-to-scan GICP ----
  Eigen::Isometry3d s2s_global_T = working_T;
  bool s2s_converged = false;

  if (prev_scan_ && prev_scan_tree_ && !prev_scan_->empty()) {
    Eigen::Isometry3d relative_guess = prev_T_.inverse() * working_T;

    small_gicp::RegistrationSetting s2s_setting;
    s2s_setting.type = small_gicp::RegistrationSetting::GICP;
    s2s_setting.max_correspondence_distance = s2s_max_correspondence_distance_;
    s2s_setting.num_threads = num_threads_;
    s2s_setting.max_iterations = s2s_max_iterations_;

    auto s2s_result = small_gicp::align(
        *prev_scan_, *source, *prev_scan_tree_, relative_guess, s2s_setting);

    s2s_converged = s2s_result.converged;

    RCLCPP_INFO(node_->get_logger(),
        "[%s] S2S: converged=%d, iter=%zu, err=%.4f, inliers=%zu",
        name_.c_str(), s2s_converged, s2s_result.iterations,
        s2s_result.error, s2s_result.num_inliers);

    if (s2s_converged) {
      s2s_global_T = prev_T_ * s2s_result.T_target_source;
    } else {
      RCLCPP_WARN(node_->get_logger(),
          "[%s] S2S did NOT converge — using IMU-predicted as S2M initial guess",
          name_.c_str());
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
        "[%s] S2S skipped: no previous scan", name_.c_str());
  }

  // ---- Step 2: Scan-to-map GICP ----
  Eigen::Isometry3d final_T = s2s_global_T;
  Eigen::Matrix<double, 6, 6> s2m_H = Eigen::Matrix<double, 6, 6>::Zero();
  bool s2m_converged = false;

  if (target && target_tree && !target->empty()) {
    small_gicp::RegistrationSetting s2m_setting;
    s2m_setting.type = small_gicp::RegistrationSetting::GICP;
    s2m_setting.max_correspondence_distance = s2m_max_correspondence_distance_;
    s2m_setting.num_threads = num_threads_;
    s2m_setting.max_iterations = s2m_max_iterations_;

    auto s2m_result = small_gicp::align(
        *target, *source, *target_tree, s2s_global_T, s2m_setting);

    s2m_converged = s2m_result.converged;
    s2m_H = s2m_result.H;

    Eigen::Vector3d s2m_pos = s2m_result.T_target_source.translation();
    Eigen::Vector3d s2s_pos = s2s_global_T.translation();
    Eigen::Vector3d correction = s2m_pos - s2s_pos;

    RCLCPP_INFO(node_->get_logger(),
        "[%s] S2M: converged=%d, iter=%zu, err=%.4f, inliers=%zu, "
        "correction=[%.4f, %.4f, %.4f]",
        name_.c_str(), s2m_converged, s2m_result.iterations,
        s2m_result.error, s2m_result.num_inliers,
        correction.x(), correction.y(), correction.z());

    if (s2m_converged) {
      final_T = s2m_result.T_target_source;
    } else {
      RCLCPP_WARN(node_->get_logger(),
          "[%s] S2M did NOT converge — falling back to S2S result",
          name_.c_str());
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
        "[%s] S2M skipped: no submap available", name_.c_str());
  }

  // Update GICP quality metrics
  last_gicp_H_ = s2m_H;
  if (!s2m_converged && target) {
    gicp_had_failure_ = true;
  }

  // Write final result to transformTobeMapped_
  Eigen::Affine3f final_af = Eigen::Affine3f::Identity();
  final_af.linear() = final_T.rotation().cast<float>();
  final_af.translation() = final_T.translation().cast<float>();
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(final_af, x, y, z, roll, pitch, yaw);
  transformTobeMapped_[0] = roll;
  transformTobeMapped_[1] = pitch;
  transformTobeMapped_[2] = yaw;
  transformTobeMapped_[3] = x;
  transformTobeMapped_[4] = y;
  transformTobeMapped_[5] = z;

  // Save Back = pose AFTER matching (for incremental odom)
  incrementalOdometryAffineBack_ = rpyxyzToAffine3f(transformTobeMapped_);

  RCLCPP_INFO(node_->get_logger(),
      "[%s] FINAL pose: [%.3f, %.3f, %.3f] (s2s_conv=%d, s2m_conv=%d)",
      name_.c_str(), transformTobeMapped_[3], transformTobeMapped_[4],
      transformTobeMapped_[5], s2s_converged, s2m_converged);
}

// ---------------------------------------------------------------------------
// publishOdometry — incremental (no hop) + global (may hop)
// ---------------------------------------------------------------------------
void LidarGICPFactor::publishOdometry(
    const builtin_interfaces::msg::Time& stamp) {

  // ---- Global odometry (may hop on correction) ----
  {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_link_frame_;
    msg.pose.pose.position.x = transformTobeMapped_[3];
    msg.pose.pose.position.y = transformTobeMapped_[4];
    msg.pose.pose.position.z = transformTobeMapped_[5];
    auto q = gtsam::Rot3::RzRyRx(transformTobeMapped_[0],
                                   transformTobeMapped_[1],
                                   transformTobeMapped_[2]).toQuaternion();
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    odom_global_pub_->publish(msg);
  }

  // ---- Incremental odometry (smooth, no hop) ----
  {
    if (!incremental_odom_initialized_) {
      // First call: seed incremental odom with current pose
      increOdomAffine_ = rpyxyzToAffine3f(transformTobeMapped_);
      incremental_odom_initialized_ = true;
    } else {
      // increOdom *= Front⁻¹ * Back (LIO-SAM line 1673-1674)
      Eigen::Affine3f affineIncre =
          incrementalOdometryAffineFront_.inverse() *
          incrementalOdometryAffineBack_;
      increOdomAffine_ = increOdomAffine_ * affineIncre;
    }

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(increOdomAffine_, x, y, z,
                                       roll, pitch, yaw);

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_link_frame_;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    auto q = gtsam::Rot3::RzRyRx(roll, pitch, yaw).toQuaternion();
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    odom_incremental_pub_->publish(msg);
  }
}

// ---------------------------------------------------------------------------
// getFactors — store latest scan in MapManager, add BetweenFactor
// ---------------------------------------------------------------------------
FactorResult LidarGICPFactor::getFactors(
    int state_index, const gtsam::Pose3& state_pose, double /*timestamp*/) {
  FactorResult result;

  RCLCPP_INFO(node_->get_logger(),
      "[%s] getFactors(state=%d, pose=[%.3f, %.3f, %.3f])",
      name_.c_str(), state_index,
      state_pose.x(), state_pose.y(), state_pose.z());

  // Store the latest single scan in MapManager (one raw scan per keyframe)
  if (latest_scan_ && !latest_scan_->empty()) {
    auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    pcl_cloud->points.resize(latest_scan_->size());
    pcl_cloud->width = static_cast<uint32_t>(latest_scan_->size());
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    for (size_t i = 0; i < latest_scan_->size(); ++i) {
      auto& p = pcl_cloud->points[i];
      p.x = static_cast<float>(latest_scan_->points[i].x());
      p.y = static_cast<float>(latest_scan_->points[i].y());
      p.z = static_cast<float>(latest_scan_->points[i].z());
      p.intensity = 0.0f;
    }

    auto& map_manager = core_->getMapManager();
    map_manager.addKeyframeData(
        state_index, "lidar_gicp_factor/cloud", pcl_cloud);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] stored %zu pts in MapManager for keyframe %d",
        name_.c_str(), latest_scan_->size(), state_index);
  } else {
    RCLCPP_WARN(node_->get_logger(),
        "[%s] getFactors: no scan available for keyframe %d (scan=%s)",
        name_.c_str(), state_index,
        latest_scan_ ? "empty" : "null");
  }

  if (state_index == 0) {
    last_keyframe_pose_ = state_pose;
    has_last_keyframe_ = true;

    // Seed transformTobeMapped_ from the initial pose
    transformTobeMapped_[0] = static_cast<float>(state_pose.rotation().roll());
    transformTobeMapped_[1] = static_cast<float>(state_pose.rotation().pitch());
    transformTobeMapped_[2] = static_cast<float>(state_pose.rotation().yaw());
    transformTobeMapped_[3] = static_cast<float>(state_pose.translation().x());
    transformTobeMapped_[4] = static_cast<float>(state_pose.translation().y());
    transformTobeMapped_[5] = static_cast<float>(state_pose.translation().z());

    // Seed incremental odom
    increOdomAffine_ = rpyxyzToAffine3f(transformTobeMapped_);
    incremental_odom_initialized_ = true;

    // Clear IMU buffer so old samples don't corrupt
    {
      std::lock_guard<std::mutex> lock(imu_odom_lock_);
      imu_odom_buffer_.clear();
      last_cloud_time_ = -1;
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] state 0: seeded transformTobeMapped, cleared IMU buffer",
        name_.c_str());
  } else if (has_last_keyframe_) {
    // Build noise model from GICP information matrix or fallback
    gtsam::SharedNoiseModel noise;
    std::string noise_type;

    if (gicp_had_failure_) {
      noise = gtsam::noiseModel::Diagonal::Variances(
          100.0 * (gtsam::Vector(6) << odom_cov_[0], odom_cov_[1], odom_cov_[2],
                    odom_cov_[3], odom_cov_[4], odom_cov_[5]).finished());
      noise_type = "100x_fallback (gicp_failure)";
      gicp_had_failure_ = false;
    } else if (last_gicp_H_.determinant() > 1e-12) {
      noise = gtsam::noiseModel::Gaussian::Information(last_gicp_H_);
      noise_type = "gicp_information_matrix (det=" +
                   std::to_string(last_gicp_H_.determinant()) + ")";
    } else {
      noise = gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << odom_cov_[0], odom_cov_[1], odom_cov_[2],
           odom_cov_[3], odom_cov_[4], odom_cov_[5]).finished());
      noise_type = "diagonal_fallback (H det=" +
                   std::to_string(last_gicp_H_.determinant()) + ")";
    }

    gtsam::Pose3 relative = last_keyframe_pose_.between(state_pose);
    auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        state_index - 1, state_index, relative, noise);
    result.factors.push_back(factor);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] BetweenFactor(%d->%d): relative=[%.3f, %.3f, %.3f], noise=%s",
        name_.c_str(), state_index - 1, state_index,
        relative.x(), relative.y(), relative.z(),
        noise_type.c_str());

    last_keyframe_pose_ = state_pose;
  }

  return result;
}

// ---------------------------------------------------------------------------
// onOptimizationComplete — correct transformTobeMapped_ from optimizer
// ---------------------------------------------------------------------------
void LidarGICPFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool loop_closure_detected) {
  if (loop_closure_detected) {
    RCLCPP_INFO(node_->get_logger(),
        "[%s] loop closure detected — clearing map cache (%zu entries)",
        name_.c_str(), map_cache_.size());
    map_cache_.clear();
  }

  int state_index = core_->getCurrentStateIndex();
  if (state_index < 0) return;
  if (!optimized_values.exists(state_index)) return;

  // Overwrite transformTobeMapped_ with latest optimized pose (LIO-SAM line 1564)
  auto pose = optimized_values.at<gtsam::Pose3>(state_index);
  transformTobeMapped_[0] = static_cast<float>(pose.rotation().roll());
  transformTobeMapped_[1] = static_cast<float>(pose.rotation().pitch());
  transformTobeMapped_[2] = static_cast<float>(pose.rotation().yaw());
  transformTobeMapped_[3] = static_cast<float>(pose.translation().x());
  transformTobeMapped_[4] = static_cast<float>(pose.translation().y());
  transformTobeMapped_[5] = static_cast<float>(pose.translation().z());

  RCLCPP_INFO(node_->get_logger(),
      "[%s] onOptimizationComplete: state=%d, corrected pose=[%.3f, %.3f, %.3f]",
      name_.c_str(), state_index,
      transformTobeMapped_[3], transformTobeMapped_[4], transformTobeMapped_[5]);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LidarGICPFactor, eidos::FactorPlugin)
