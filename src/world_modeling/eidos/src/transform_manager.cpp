#include "eidos/core/transform_manager.hpp"

namespace eidos {

void TransformManager::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const PluginRegistry* registry,
    const LockFreePose* estimator_pose,
    const std::string& map_frame,
    const std::string& odom_frame,
    const std::string& base_link_frame,
    const std::string& odom_source_name,
    const std::string& map_source_name,
    double rate_hz) {
  node_ = node;
  registry_ = registry;
  estimator_pose_ = estimator_pose;
  map_frame_ = map_frame;
  odom_frame_ = odom_frame;
  base_link_frame_ = base_link_frame;
  odom_source_name_ = odom_source_name;
  map_source_name_ = map_source_name;

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Configure EKF if odom_source is set
  if (!odom_source_name_.empty()) {
    // Read fusion noise params — must be in config
    std::vector<double> process_noise, measurement_noise;
    node_->get_parameter("transforms.fusion.process_noise", process_noise);
    node_->get_parameter("transforms.fusion.measurement_noise", measurement_noise);

    Eigen::Matrix<double, 6, 1> Q_diag, R_diag;
    // Config is [roll,pitch,yaw,x,y,z], same order as GTSAM Pose3 Logmap
    for (int i = 0; i < 6 && i < static_cast<int>(process_noise.size()); i++)
      Q_diag(i) = process_noise[i];
    for (int i = 0; i < 6 && i < static_cast<int>(measurement_noise.size()); i++)
      R_diag(i) = measurement_noise[i];

    odom_ekf_.configure(Q_diag, R_diag);
  }

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TransformManager::tick, this),
      callback_group_);
  timer_->cancel();
}

void TransformManager::activate() {
  odom_pub_->on_activate();
  timer_->reset();
}

void TransformManager::deactivate() {
  timer_->cancel();
  odom_pub_->on_deactivate();
}

void TransformManager::tick() {
  auto now = node_->now();

  // 1. Compute and broadcast odom → base_link
  cached_odom_to_base_ = computeOdomToBase();
  broadcastTf(now, odom_frame_, base_link_frame_, cached_odom_to_base_);

  // 1b. Publish unified fused odometry
  if (odom_pub_->is_activated()) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    auto q = cached_odom_to_base_.rotation().toQuaternion();
    odom_msg.pose.pose.position.x = cached_odom_to_base_.translation().x();
    odom_msg.pose.pose.position.y = cached_odom_to_base_.translation().y();
    odom_msg.pose.pose.position.z = cached_odom_to_base_.translation().z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg);
  }

  // 2. Read map-frame pose from configured source (lock-free)
  //    When map_source is empty (localization mode), map→odom is fixed from
  //    relocalization via setMapToOdom() and never recomputed.
  std::optional<gtsam::Pose3> map_pose;
  if (!map_source_name_.empty()) {
    if (map_source_name_ == "slam_core") {
      if (estimator_pose_) {
        map_pose = estimator_pose_->load();
      }
    } else {
      auto src = registry_->findFactor(map_source_name_);
      if (src) {
        map_pose = src->getMapPose();
      }
    }

    // 3. map→odom only changes when map_src changes (SLAM optimization).
    //    Between optimizations, map→odom is constant — odom→base carries all motion.
    if (map_pose) {
      if (!has_last_map_pose_ || !map_pose->equals(last_map_pose_, 1e-12)) {
        cached_map_to_odom_ = map_pose->compose(cached_odom_to_base_.inverse());
        last_map_pose_ = *map_pose;
        has_last_map_pose_ = true;
      }
    }
  }

  broadcastTf(now, map_frame_, odom_frame_, cached_map_to_odom_);

  // Write current map-frame robot pose (for viz, publishing, etc.)
  current_map_pose_.store(cached_map_to_odom_.compose(cached_odom_to_base_));
}

gtsam::Pose3 TransformManager::computeOdomToBase() {
  if (!registry_->motion_model) return cached_odom_to_base_;
  auto mm_pose = registry_->motion_model->getOdomPose();
  if (!mm_pose) return cached_odom_to_base_;

  // No odom_source — motion model IS the odom source (no EKF)
  if (odom_source_name_.empty()) {
    return *mm_pose;
  }

  // ---- EKF fusion: MM prediction + odom_source correction ----

  // Prediction step: compute MM delta since last tick
  if (has_last_mm_) {
    gtsam::Pose3 mm_delta = last_mm_pose_.between(*mm_pose);
    odom_ekf_.predict(mm_delta);
  } else {
    // First MM reading — initialize EKF from MM pose
    odom_ekf_.reset(*mm_pose);
  }
  last_mm_pose_ = *mm_pose;
  has_last_mm_ = true;

  // Measurement update: check if odom_source has new data
  auto src = registry_->findFactor(odom_source_name_);
  if (src) {
    auto odom_reading = src->getOdomPose();
    if (odom_reading) {
      if (!has_odom_source_reading_ ||
          !odom_reading->equals(last_odom_source_pose_, 1e-12)) {
        odom_ekf_.update(*odom_reading);
        last_odom_source_pose_ = *odom_reading;
        has_odom_source_reading_ = true;
      }
    }
  }

  return odom_ekf_.pose();
}

void TransformManager::broadcastTf(
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const gtsam::Pose3& pose) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;

  auto q = pose.rotation().toQuaternion();
  tf.transform.translation.x = pose.translation().x();
  tf.transform.translation.y = pose.translation().y();
  tf.transform.translation.z = pose.translation().z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  broadcaster_->sendTransform(tf);
}

}  // namespace eidos
