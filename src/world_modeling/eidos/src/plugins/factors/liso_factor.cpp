#include "eidos/plugins/factors/liso_factor.hpp"

#include <queue>
#include <thread>
#include <unordered_set>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <small_gicp/registration/registration_helper.hpp>

#include "eidos/slam_core.hpp"
#include "eidos/utils/small_gicp_ros.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void LisoFactor::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // Declare parameters
  node_->declare_parameter(prefix + ".lidar_topic", "/lidar/points");
  node_->declare_parameter(prefix + ".odom_topic", "liso/odometry");
  node_->declare_parameter(prefix + ".odometry_incremental_topic", "liso/odometry_incremental");
  node_->declare_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->declare_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->declare_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->declare_parameter(prefix + ".submap_radius", submap_radius_);
  node_->declare_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->declare_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->declare_parameter(prefix + ".max_iterations", max_iterations_);
  node_->declare_parameter(prefix + ".num_threads", num_threads_);
  node_->declare_parameter(prefix + ".min_inliers", min_inliers_);
  node_->declare_parameter(prefix + ".min_noise", min_noise_);
  node_->declare_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->declare_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->declare_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);
  node_->declare_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->declare_parameter(prefix + ".imu_topic", imu_topic_);
  node_->declare_parameter(prefix + ".imu_frame", imu_frame_);
  node_->declare_parameter(prefix + ".initialization.warmup_samples", imu_warmup_samples_);
  node_->declare_parameter(prefix + ".initialization.stationary_gyr_threshold", imu_stationary_gyr_threshold_);

  // Read parameters
  std::string lidar_topic, odom_topic, odometry_incremental_topic;
  node_->get_parameter(prefix + ".lidar_topic", lidar_topic);
  node_->get_parameter(prefix + ".odom_topic", odom_topic);
  node_->get_parameter(prefix + ".odometry_incremental_topic", odometry_incremental_topic);
  node_->get_parameter(prefix + ".scan_ds_resolution", scan_ds_resolution_);
  node_->get_parameter(prefix + ".submap_ds_resolution", submap_ds_resolution_);
  node_->get_parameter(prefix + ".num_neighbors", num_neighbors_);
  node_->get_parameter(prefix + ".submap_radius", submap_radius_);
  node_->get_parameter(prefix + ".max_submap_states", max_submap_states_);
  node_->get_parameter(prefix + ".max_correspondence_distance", max_correspondence_distance_);
  node_->get_parameter(prefix + ".max_iterations", max_iterations_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".min_inliers", min_inliers_);
  node_->get_parameter(prefix + ".min_noise", min_noise_);
  node_->get_parameter(prefix + ".min_scan_distance", min_scan_distance_);
  node_->get_parameter(prefix + ".odom_pose_cov", odom_pose_cov_);
  node_->get_parameter(prefix + ".odom_twist_cov", odom_twist_cov_);
  node_->get_parameter(prefix + ".lidar_frame", lidar_frame_);
  node_->get_parameter(prefix + ".imu_topic", imu_topic_);
  node_->get_parameter(prefix + ".imu_frame", imu_frame_);
  node_->get_parameter(prefix + ".initialization.warmup_samples", imu_warmup_samples_);
  node_->get_parameter(prefix + ".initialization.stationary_gyr_threshold", imu_stationary_gyr_threshold_);

  node_->get_parameter("frames.map", map_frame_);
  node_->get_parameter("frames.odometry", odom_frame_);
  node_->get_parameter("frames.base_link", base_link_frame_);

  // Create subscription
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&LisoFactor::lidarCallback, this, std::placeholders::_1),
      sub_opts);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LisoFactor::imuCallback, this, std::placeholders::_1),
      sub_opts);

  // Create odometry publishers
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  odom_incremental_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odometry_incremental_topic, 10);
  submap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "liso/submap", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (GICP scan-to-submap)", name_.c_str());
}

void LisoFactor::activate() {
  active_ = true;
  odom_pub_->on_activate();
  odom_incremental_pub_->on_activate();
  submap_pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated (publish_tf=%s)", name_.c_str(),
              publishesTf() ? "true" : "false");
}

void LisoFactor::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void LisoFactor::reset() {
  RCLCPP_INFO(node_->get_logger(), "[%s] reset", name_.c_str());
  first_scan_ = true;
  scan_received_ = false;
  has_tf_ = false;
  has_imu_tf_ = false;
  has_last_factor_ = false;
  has_prev_liso_ = false;
  has_last_match_ = false;
  incremental_pose_ = gtsam::Pose3();
  prev_incremental_pose_ = gtsam::Pose3();
  has_prev_incremental_ = false;
  prev_odom_time_ = 0.0;
  gyro_tracking_active_ = false;
  gyro_delta_rpy_ = Eigen::Vector3d::Zero();
  last_gyro_time_ = 0.0;
  imu_warmup_complete_ = false;
  imu_warmup_count_ = 0;
  warmup_quat_sum_ = Eigen::Vector4d::Zero();
  warmup_quat_hemisphere_set_ = false;

  {
    std::lock_guard lock(imu_mtx_);
    imu_buffer_.clear();
  }
  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_.reset();
    cached_submap_tree_.reset();
  }
  {
    std::lock_guard lock(result_mtx_);
    has_cached_result_ = false;
  }
}

bool LisoFactor::isReady() const {
  return scan_received_ && imu_warmup_complete_;
}

std::string LisoFactor::getReadyStatus() const {
  if (!imu_warmup_complete_) return "IMU warmup (stationarity + gravity alignment)";
  if (!scan_received_) return "waiting for LiDAR data";
  return "ready";
}

bool LisoFactor::hasData() const {
  std::lock_guard lock(result_mtx_);
  return has_cached_result_;
}

// ---------------------------------------------------------------------------
// LiDAR callback — scan matching + odometry publishing (~27 Hz)
// ---------------------------------------------------------------------------
void LisoFactor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!active_) return;

  if (!scan_received_) {
    scan_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] first LiDAR scan received (%zu points)",
                name_.c_str(), static_cast<size_t>(msg->width * msg->height));
  }

  // Don't process scans until TRACKING — IMU needs to calibrate gravity first
  if (core_->getState() != SlamState::TRACKING) return;

  // Look up static TF: base_link <- lidar (once)
  if (!has_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, lidar_frame_,
                                          tf2::TimePointZero);
      const auto& t = tf_msg.transform.translation;
      const auto& r = tf_msg.transform.rotation;
      T_base_lidar_ = Eigen::Isometry3d::Identity();
      T_base_lidar_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      T_base_lidar_.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_tf_ = true;
      RCLCPP_INFO(node_->get_logger(), "[%s] got TF %s <- %s",
                  name_.c_str(), base_link_frame_.c_str(), lidar_frame_.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "[%s] waiting for TF %s <- %s: %s",
                            name_.c_str(), base_link_frame_.c_str(),
                            lidar_frame_.c_str(), ex.what());
      return;
    }
  }

  // Convert ROS msg to small_gicp cloud
  auto raw_cloud = fromRosMsg(*msg);
  if (raw_cloud->empty()) return;

  // Transform points from lidar frame to body frame BEFORE preprocessing,
  // so normals and covariances are computed in body frame (consistent with
  // the frame used for scan matching). Without this, GICP uses lidar-frame
  // normals against world-frame submap normals, causing systematic drift.
  auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  pcl_cloud->reserve(raw_cloud->size());
  for (size_t i = 0; i < raw_cloud->size(); i++) {
    Eigen::Vector4d& pt = raw_cloud->point(i);
    Eigen::Vector3d p = T_base_lidar_ * pt.head<3>();
    pt.head<3>() = p;

    PointType pcl_pt;
    pcl_pt.x = static_cast<float>(p.x());
    pcl_pt.y = static_cast<float>(p.y());
    pcl_pt.z = static_cast<float>(p.z());
    pcl_pt.intensity = 0.0f;
    pcl_cloud->push_back(pcl_pt);
  }

  // Preprocess: downsample + normals + covariances (now in body frame)
  auto [scan, scan_tree] = small_gicp::preprocess_points(
      *raw_cloud, scan_ds_resolution_, num_neighbors_, num_threads_);

  if (scan->empty()) return;

  double scan_time = stamp2Sec(msg->header.stamp);

  // First scan: no submap to match against — just cache
  if (first_scan_) {
    std::lock_guard lock(result_mtx_);
    cached_cloud_ = scan;
    cached_pcl_cloud_ = pcl_cloud;
    cached_timestamp_ = scan_time;

    // Seed from warmup gravity alignment (guaranteed by isReady() gate)
    cached_pose_ = gtsam::Pose3(initial_gravity_orientation_, gtsam::Point3(0, 0, 0));
    cached_result_ = small_gicp::RegistrationResult();
    cached_result_.converged = true;
    cached_result_.num_inliers = scan->size();
    // Set H to a reasonable default for the first scan
    cached_result_.H = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
    has_cached_result_ = true;
    first_scan_ = false;

    // Seed last match tracking for initial guess computation
    last_matched_pose_ = cached_pose_;
    has_last_match_ = true;

    // Start gyro tracking from this scan time
    gyro_tracking_active_ = true;
    gyro_delta_rpy_ = Eigen::Vector3d::Zero();
    last_gyro_time_ = scan_time;

    RCLCPP_INFO(node_->get_logger(), "[%s] first scan cached (%zu points), awaiting submap",
                name_.c_str(), scan->size());
    return;
  }

  // Initial guess: last GICP pose + raw gyro-integrated rotation delta
  // Independent of motion model — avoids feedback loop through graph optimization
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (has_last_match_) {
    // gyro_delta_rpy_ has been accumulating since last successful match
    gtsam::Rot3 delta_rot = gtsam::Rot3::RzRyRx(gyro_delta_rpy_);
    gtsam::Rot3 init_rotation = last_matched_pose_.rotation() * delta_rot;
    gtsam::Point3 init_translation = last_matched_pose_.translation();
    gtsam::Pose3 predicted(init_rotation, init_translation);
    init_guess = Eigen::Isometry3d(predicted.matrix());
  }

  // Scan-to-submap matching
  small_gicp::RegistrationResult result;
  size_t submap_size = 0;
  {
    std::shared_lock lock(submap_mtx_);
    if (!cached_submap_ || !cached_submap_tree_ || cached_submap_->empty()) {
      return;  // Submap not yet built
    }
    submap_size = cached_submap_->size();

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = max_iterations_;
    setting.num_threads = num_threads_;

    result = small_gicp::align(
        *cached_submap_, *scan, *cached_submap_tree_, init_guess, setting);
  }

  // Fitness check
  if (!result.converged || static_cast<int>(result.num_inliers) < min_inliers_) {
    gtsam::Pose3 gicp_pose(gtsam::Rot3(result.T_target_source.rotation()),
                            gtsam::Point3(result.T_target_source.translation()));
    RCLCPP_WARN(node_->get_logger(),
        "[%s] REJECTED: converged=%d inliers=%zu submap=%zu | "
        "init=(%.2f,%.2f,%.2f) result=(%.2f,%.2f,%.2f) gyro=(%.4f,%.4f,%.4f)",
        name_.c_str(), result.converged, result.num_inliers, submap_size,
        init_guess.translation().x(), init_guess.translation().y(), init_guess.translation().z(),
        gicp_pose.translation().x(), gicp_pose.translation().y(), gicp_pose.translation().z(),
        gyro_delta_rpy_(0), gyro_delta_rpy_(1), gyro_delta_rpy_(2));
    return;
  }

  // Extract matched pose
  gtsam::Pose3 matched_pose(gtsam::Rot3(result.T_target_source.rotation()),
                             gtsam::Point3(result.T_target_source.translation()));

  // Cache result for getFactors only when the distance gate is satisfied.
  // This runs at LiDAR rate, so the factor is locked in at the exact scan
  // where min_scan_distance is crossed — independent of SlamCore's poll rate.
  {
    std::lock_guard lock(result_mtx_);
    bool should_cache = false;
    if (!has_last_factor_) {
      should_cache = true;  // first factor always ready
    } else {
      double dist = (matched_pose.translation() - last_factor_position_).norm();
      should_cache = (dist >= min_scan_distance_);
    }
    if (should_cache && !has_cached_result_) {
      cached_result_ = result;
      cached_pose_ = matched_pose;
      cached_cloud_ = scan;
      cached_pcl_cloud_ = pcl_cloud;
      cached_timestamp_ = scan_time;
      has_cached_result_ = true;
    }
  }

  // Track last successful match for initial guess computation
  last_matched_pose_ = matched_pose;
  has_last_match_ = true;

  // Reset gyro accumulator — next initial guess starts from this matched pose
  gyro_delta_rpy_ = Eigen::Vector3d::Zero();
  last_gyro_time_ = scan_time;

  // Compute scan-to-scan delta for incremental odometry and twist
  gtsam::Pose3 delta;
  bool has_delta = false;
  double dt = (prev_odom_time_ > 0.0) ? (scan_time - prev_odom_time_) : 0.0;

  if (has_prev_incremental_) {
    delta = prev_incremental_pose_.between(matched_pose);
    incremental_pose_ = incremental_pose_.compose(delta);
    has_delta = true;
  } else {
    // First match — seed incremental pose from gravity-aligned initial
    incremental_pose_ = matched_pose;
    has_prev_incremental_ = true;
  }
  prev_incremental_pose_ = matched_pose;
  prev_odom_time_ = scan_time;

  // Twist: body-frame velocity from scan-to-scan delta
  bool publish_twist = has_delta && dt > 0.0 && dt < 1.0;

  // Publish map-frame odometry (absolute, includes corrections)
  if (odom_pub_->is_activated()) {
    auto q = matched_pose.rotation().toQuaternion();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = matched_pose.translation().x();
    odom_msg.pose.pose.position.y = matched_pose.translation().y();
    odom_msg.pose.pose.position.z = matched_pose.translation().z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.pose.covariance[0]  = odom_pose_cov_[0];
    odom_msg.pose.covariance[7]  = odom_pose_cov_[1];
    odom_msg.pose.covariance[14] = odom_pose_cov_[2];
    odom_msg.pose.covariance[21] = odom_pose_cov_[3];
    odom_msg.pose.covariance[28] = odom_pose_cov_[4];
    odom_msg.pose.covariance[35] = odom_pose_cov_[5];

    if (publish_twist) {
      // delta is in prev body frame; for small dt at LiDAR rate this ≈ current body frame
      gtsam::Point3 t = delta.translation();
      gtsam::Vector3 rpy = delta.rotation().rpy();
      odom_msg.twist.twist.linear.x = t.x() / dt;
      odom_msg.twist.twist.linear.y = t.y() / dt;
      odom_msg.twist.twist.linear.z = t.z() / dt;
      odom_msg.twist.twist.angular.x = rpy(0) / dt;
      odom_msg.twist.twist.angular.y = rpy(1) / dt;
      odom_msg.twist.twist.angular.z = rpy(2) / dt;

      odom_msg.twist.covariance[0]  = odom_twist_cov_[0];
      odom_msg.twist.covariance[7]  = odom_twist_cov_[1];
      odom_msg.twist.covariance[14] = odom_twist_cov_[2];
      odom_msg.twist.covariance[21] = odom_twist_cov_[3];
      odom_msg.twist.covariance[28] = odom_twist_cov_[4];
      odom_msg.twist.covariance[35] = odom_twist_cov_[5];
    }

    odom_pub_->publish(odom_msg);
  }

  // Publish odom-frame odometry (incremental, never corrected — smooth)
  if (odom_incremental_pub_->is_activated()) {
    auto q_inc = incremental_pose_.rotation().toQuaternion();

    nav_msgs::msg::Odometry odom_inc_msg;
    odom_inc_msg.header.stamp = msg->header.stamp;
    odom_inc_msg.header.frame_id = odom_frame_;
    odom_inc_msg.child_frame_id = base_link_frame_;
    odom_inc_msg.pose.pose.position.x = incremental_pose_.translation().x();
    odom_inc_msg.pose.pose.position.y = incremental_pose_.translation().y();
    odom_inc_msg.pose.pose.position.z = incremental_pose_.translation().z();
    odom_inc_msg.pose.pose.orientation.x = q_inc.x();
    odom_inc_msg.pose.pose.orientation.y = q_inc.y();
    odom_inc_msg.pose.pose.orientation.z = q_inc.z();
    odom_inc_msg.pose.pose.orientation.w = q_inc.w();

    odom_inc_msg.pose.covariance[0]  = odom_pose_cov_[0];
    odom_inc_msg.pose.covariance[7]  = odom_pose_cov_[1];
    odom_inc_msg.pose.covariance[14] = odom_pose_cov_[2];
    odom_inc_msg.pose.covariance[21] = odom_pose_cov_[3];
    odom_inc_msg.pose.covariance[28] = odom_pose_cov_[4];
    odom_inc_msg.pose.covariance[35] = odom_pose_cov_[5];

    if (publish_twist) {
      gtsam::Point3 t = delta.translation();
      gtsam::Vector3 rpy = delta.rotation().rpy();
      odom_inc_msg.twist.twist.linear.x = t.x() / dt;
      odom_inc_msg.twist.twist.linear.y = t.y() / dt;
      odom_inc_msg.twist.twist.linear.z = t.z() / dt;
      odom_inc_msg.twist.twist.angular.x = rpy(0) / dt;
      odom_inc_msg.twist.twist.angular.y = rpy(1) / dt;
      odom_inc_msg.twist.twist.angular.z = rpy(2) / dt;

      odom_inc_msg.twist.covariance[0]  = odom_twist_cov_[0];
      odom_inc_msg.twist.covariance[7]  = odom_twist_cov_[1];
      odom_inc_msg.twist.covariance[14] = odom_twist_cov_[2];
      odom_inc_msg.twist.covariance[21] = odom_twist_cov_[3];
      odom_inc_msg.twist.covariance[28] = odom_twist_cov_[4];
      odom_inc_msg.twist.covariance[35] = odom_twist_cov_[5];
    }

    odom_incremental_pub_->publish(odom_inc_msg);
  }

  // Broadcast odom -> base_link TF at LiDAR rate
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_link_frame_;
    auto q_tf = incremental_pose_.rotation().toQuaternion();
    tf.transform.translation.x = incremental_pose_.translation().x();
    tf.transform.translation.y = incremental_pose_.translation().y();
    tf.transform.translation.z = incremental_pose_.translation().z();
    tf.transform.rotation.x = q_tf.x();
    tf.transform.rotation.y = q_tf.y();
    tf.transform.rotation.z = q_tf.z();
    tf.transform.rotation.w = q_tf.w();
    sendTransform(tf);
  }
}

// ---------------------------------------------------------------------------
// IMU callback — buffer messages + integrate gyro for initial guess
// ---------------------------------------------------------------------------
void LisoFactor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  double current_time = stamp2Sec(msg->header.stamp);

  // Look up static TF: base_link <- imu_link (once, needed for warmup + gyro integration)
  if (!has_imu_tf_) {
    try {
      auto tf_msg = tf_->lookupTransform(base_link_frame_, imu_frame_,
                                          tf2::TimePointZero);
      const auto& r = tf_msg.transform.rotation;
      R_base_imu_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
      has_imu_tf_ = true;
      RCLCPP_INFO(node_->get_logger(), "[%s] got TF %s <- %s (IMU)",
                  name_.c_str(), base_link_frame_.c_str(), imu_frame_.c_str());
    } catch (const tf2::TransformException&) {
      return;  // can't do anything without IMU TF
    }
  }

  // Buffer IMU messages (keep ~5s for potential future use)
  {
    std::lock_guard lock(imu_mtx_);
    imu_buffer_.push_back(*msg);
    while (imu_buffer_.size() > 2500) {  // ~5s at 500Hz
      imu_buffer_.pop_front();
    }
  }

  // Warmup: stationarity detection + averaged gravity alignment from IMU orientation
  if (!imu_warmup_complete_) {
    bool has_valid_orientation = msg->orientation_covariance[0] >= 0.0;
    double gyr_mag = std::sqrt(
        msg->angular_velocity.x * msg->angular_velocity.x +
        msg->angular_velocity.y * msg->angular_velocity.y +
        msg->angular_velocity.z * msg->angular_velocity.z);

    if (gyr_mag < imu_stationary_gyr_threshold_ && has_valid_orientation) {
      Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                           msg->orientation.y, msg->orientation.z);
      if (q.squaredNorm() > 0.5) {
        q.normalize();

        // Ensure hemisphere consistency: flip q if it's in the opposite hemisphere
        // from the first sample (q and -q represent the same rotation)
        if (!warmup_quat_hemisphere_set_) {
          warmup_quat_reference_ = q;
          warmup_quat_hemisphere_set_ = true;
        } else if (q.dot(warmup_quat_reference_) < 0.0) {
          q.coeffs() = -q.coeffs();
        }

        warmup_quat_sum_ += Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        imu_warmup_count_++;
      }
    } else {
      // Motion detected — reset accumulation
      imu_warmup_count_ = 0;
      warmup_quat_sum_ = Eigen::Vector4d::Zero();
      warmup_quat_hemisphere_set_ = false;
    }

    if (imu_warmup_count_ >= imu_warmup_samples_) {
      // Average quaternion over the stationary window
      Eigen::Vector4d avg = warmup_quat_sum_ / imu_warmup_count_;
      Eigen::Quaterniond q_avg(avg(0), avg(1), avg(2), avg(3));
      q_avg.normalize();

      // R_world_base = R_world_imu * R_base_imu^T
      Eigen::Matrix3d R_world_imu = q_avg.toRotationMatrix();
      Eigen::Matrix3d R_world_base = R_world_imu * R_base_imu_.transpose();

      // Extract only gravity alignment (roll, pitch), zero yaw.
      // Map frame heading is arbitrary at startup — GPS provides absolute heading
      // via R_map_enu. Including yaw here would double-count the heading.
      gtsam::Rot3 full_rot(R_world_base);
      initial_gravity_orientation_ = gtsam::Rot3::RzRyRx(0.0, full_rot.pitch(), full_rot.roll());

      imu_warmup_complete_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[%s] IMU warmup complete — gravity aligned from %d samples "
                  "(roll: %.4f, pitch: %.4f, yaw: 0.0)",
                  name_.c_str(), imu_warmup_count_,
                  initial_gravity_orientation_.roll(),
                  initial_gravity_orientation_.pitch());
    }
    last_gyro_time_ = current_time;
    return;  // don't integrate gyro until warmup is done
  }

  // Integrate angular velocity for initial guess rotation tracking
  if (gyro_tracking_active_ && last_gyro_time_ > 0.0) {
    double dt = current_time - last_gyro_time_;
    if (dt > 0.0 && dt < 0.1) {  // sanity check: skip if dt is negative or too large
      // Transform angular velocity from IMU frame to base_link frame
      Eigen::Vector3d omega_imu(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);
      Eigen::Vector3d omega_base = R_base_imu_ * omega_imu;
      gyro_delta_rpy_ += omega_base * dt;  // Euler integration (fine for small dt at 500Hz)
    }
  }
  last_gyro_time_ = current_time;
}

// ---------------------------------------------------------------------------
// processFrame — return cached GICP pose
// ---------------------------------------------------------------------------
std::optional<gtsam::Pose3> LisoFactor::processFrame(double /*timestamp*/) {
  std::lock_guard lock(result_mtx_);
  if (has_cached_result_) {
    return cached_pose_;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// getFactors — return BetweenFactor from cached GICP result
// ---------------------------------------------------------------------------
StampedFactorResult LisoFactor::getFactors(gtsam::Key key) {
  StampedFactorResult result;
  if (!active_) return result;

  std::lock_guard lock(result_mtx_);
  if (!has_cached_result_) return result;

  // Consume the cached result
  has_cached_result_ = false;
  gtsam::Point3 current_pos = cached_pose_.translation();

  // Every LISO state follows the same path: timestamp + initial value + clouds.
  // SlamCore handles PriorFactor for the very first state.
  result.timestamp = cached_timestamp_;
  result.values.insert(key, cached_pose_);

  // Store body-frame clouds in MapManager
  core_->getMapManager().addKeyframeData(key, "liso_factor/gicp_cloud", cached_cloud_);
  core_->getMapManager().addKeyframeData(key, "liso_factor/cloud", cached_pcl_cloud_);

  // BetweenFactor: relative LiDAR odometry from previous LISO state
  if (has_prev_liso_) {
    // Fixed covariance — config is [x, y, z, roll, pitch, yaw],
    // GTSAM expects [rot, rot, rot, trans, trans, trans]
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << odom_pose_cov_[3], odom_pose_cov_[4], odom_pose_cov_[5],
         odom_pose_cov_[0], odom_pose_cov_[1], odom_pose_cov_[2]).finished());
    gtsam::Pose3 relative_pose = prev_liso_pose_.between(cached_pose_);
    auto between_factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        prev_liso_key_, key, relative_pose, noise);
    result.factors.push_back(between_factor);
  }

  // Update tracking
  last_factor_position_ = current_pos;
  has_last_factor_ = true;
  prev_liso_key_ = key;
  prev_liso_pose_ = cached_pose_;
  has_prev_liso_ = true;

  gtsam::Symbol sym(key);
  RCLCPP_INFO(node_->get_logger(),
              "\033[36m[%s] %s at (%c,%lu) t=%.3f inliers=%zu\033[0m",
              name_.c_str(),
              result.factors.empty() ? "origin" : "BetweenFactor",
              sym.chr(), sym.index(), cached_timestamp_,
              cached_result_.num_inliers);

  return result;
}

// ---------------------------------------------------------------------------
// onOptimizationComplete — rebuild submap with corrected poses
// ---------------------------------------------------------------------------
void LisoFactor::onOptimizationComplete(
    const gtsam::Values& optimized_values, bool /*loop_closure_detected*/) {
  // Update prev_liso_pose_ and last_matched_pose_ to the optimized values.
  // After a GPS correction, ISAM2 shifts all poses. If we keep stale poses,
  // the next BetweenFactor's relative_pose = prev.between(new) bakes in the
  // correction as fake motion, and the GICP initial guess is wrong.
  if (has_prev_liso_ && optimized_values.exists(prev_liso_key_)) {
    gtsam::Pose3 corrected = optimized_values.at<gtsam::Pose3>(prev_liso_key_);
    double d_pos = (corrected.translation() - prev_liso_pose_.translation()).norm();
    double d_rot = gtsam::Rot3::Logmap(
        prev_liso_pose_.rotation().between(corrected.rotation())).norm();
    if (d_pos > 0.01 || d_rot > 0.001) {
      gtsam::Symbol sym(prev_liso_key_);
      RCLCPP_INFO(node_->get_logger(),
          "\033[35m[%s] correction (%c,%lu): d_pos=%.3fm d_rot=%.2f° | "
          "new=(%.2f,%.2f,%.2f) rebuild_in_progress=%d\033[0m",
          name_.c_str(), sym.chr(), sym.index(),
          d_pos, d_rot * 180.0 / M_PI,
          corrected.translation().x(), corrected.translation().y(),
          corrected.translation().z(), rebuild_in_progress_.load());
    }

    prev_liso_pose_ = corrected;
    last_matched_pose_ = corrected;
    prev_incremental_pose_ = corrected;
  }

  // Dispatch submap rebuild off the SLAM thread.  submap_mtx_ (shared_lock
  // in lidarCallback, unique_lock in rebuildSubmap) handles concurrency.
  if (!rebuild_in_progress_.exchange(true)) {
    std::thread([this]() {
      rebuildSubmap();
      rebuild_in_progress_ = false;
    }).detach();
  }
}

// ---------------------------------------------------------------------------
// rebuildSubmap — graph BFS from latest state, assemble world-frame submap
// ---------------------------------------------------------------------------
void LisoFactor::rebuildSubmap() {
  if (core_->getState() != SlamState::TRACKING) return;
  auto t0 = std::chrono::steady_clock::now();
  auto ms_since = [&t0]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  };

  const auto& map_manager = core_->getMapManager();
  auto key_list = map_manager.getKeyList();
  if (key_list.empty()) return;

  // Start BFS from the most recent key
  gtsam::Key start_key = key_list.back();

  auto collected_keys = bfsCollectStates(start_key, submap_radius_);
  if (collected_keys.empty()) return;
  double t_bfs = ms_since();

  // Assemble world-frame submap
  auto merged = std::make_shared<small_gicp::PointCloud>();
  auto poses_6d = map_manager.getKeyPoses6D();
  size_t total_raw_points = 0;

  for (gtsam::Key k : collected_keys) {
    auto cloud_data = map_manager.getKeyframeData(k, "liso_factor/gicp_cloud");
    if (!cloud_data.has_value()) continue;

    small_gicp::PointCloud::Ptr body_cloud;
    try {
      body_cloud = std::any_cast<small_gicp::PointCloud::Ptr>(cloud_data.value());
    } catch (const std::bad_any_cast&) {
      continue;
    }
    if (!body_cloud || body_cloud->empty()) continue;

    // Get optimized pose for this key
    int idx = map_manager.getCloudIndex(k);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    const auto& pose = poses_6d->points[idx];
    Eigen::Affine3f world_T = poseTypeToAffine3f(pose);
    Eigen::Isometry3d world_T_d;
    world_T_d.matrix() = world_T.matrix().cast<double>();

    // Transform body-frame cloud to world frame
    total_raw_points += body_cloud->size();
    for (size_t i = 0; i < body_cloud->size(); i++) {
      Eigen::Vector3d p = world_T_d * body_cloud->point(i).head<3>();
      merged->points.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
  }

  if (merged->empty()) return;
  double t_assemble = ms_since();

  // Preprocess merged submap: downsample + normals + covariances + KdTree
  auto [submap, submap_tree] = small_gicp::preprocess_points(
      *merged, submap_ds_resolution_, num_neighbors_, num_threads_);
  double t_preprocess = ms_since();

  // Update cached submap under exclusive lock
  {
    std::unique_lock lock(submap_mtx_);
    cached_submap_ = submap;
    cached_submap_tree_ = submap_tree;
  }

  RCLCPP_INFO(node_->get_logger(),
      "\033[33m[SUBMAP]\033[0m states=%zu raw_pts=%zu merged=%zu final=%zu | "
      "bfs=%.1fms assemble=%.1fms preprocess=%.1fms total=%.1fms",
      collected_keys.size(), total_raw_points, merged->size(), submap->size(),
      t_bfs, t_assemble - t_bfs, t_preprocess - t_assemble, t_preprocess);

  // Publish submap for visualization
  if (submap_pub_ && submap_pub_->is_activated()) {
    pcl::PointCloud<pcl::PointXYZ> pcl_submap;
    pcl_submap.reserve(submap->size());
    for (size_t i = 0; i < submap->size(); i++) {
      const auto& p = submap->point(i);
      pcl_submap.emplace_back(p.x(), p.y(), p.z());
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl_submap, msg);
    msg.header.stamp = node_->now();
    msg.header.frame_id = map_frame_;
    submap_pub_->publish(msg);
  }
}

// ---------------------------------------------------------------------------
// bfsCollectStates — BFS over accumulated graph, radial distance bounded
// ---------------------------------------------------------------------------
std::vector<gtsam::Key> LisoFactor::bfsCollectStates(
    gtsam::Key start, double radius) {
  const auto& map_manager = core_->getMapManager();
  auto poses_6d = map_manager.getKeyPoses6D();

  // Use incrementally-maintained adjacency from MapManager (O(1) lookup)
  const auto& adjacency = map_manager.getAdjacency();

  // Get start position for distance check
  int start_idx = map_manager.getCloudIndex(start);
  if (start_idx < 0) return {};
  const auto& start_pose = poses_6d->points[start_idx];
  Eigen::Vector3f start_pos(start_pose.x, start_pose.y, start_pose.z);

  // BFS
  std::vector<gtsam::Key> collected;
  std::unordered_set<gtsam::Key> visited;
  std::queue<gtsam::Key> frontier;

  frontier.push(start);
  visited.insert(start);

  while (!frontier.empty() &&
         static_cast<int>(collected.size()) < max_submap_states_) {
    gtsam::Key current = frontier.front();
    frontier.pop();

    // Check if this key has a pose in MapManager
    int idx = map_manager.getCloudIndex(current);
    if (idx < 0 || idx >= static_cast<int>(poses_6d->size())) continue;

    // Check radial distance
    const auto& current_pose = poses_6d->points[idx];
    Eigen::Vector3f current_pos(current_pose.x, current_pose.y, current_pose.z);
    float dist = (current_pos - start_pos).norm();
    if (dist > radius) continue;

    collected.push_back(current);

    // Expand neighbors
    auto it = adjacency.find(current);
    if (it == adjacency.end()) continue;
    for (gtsam::Key neighbor : it->second) {
      if (visited.count(neighbor) == 0) {
        visited.insert(neighbor);
        frontier.push(neighbor);
      }
    }
  }

  return collected;
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::LisoFactor, eidos::FactorPlugin)
