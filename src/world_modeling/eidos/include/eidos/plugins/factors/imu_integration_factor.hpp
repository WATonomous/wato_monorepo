#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Geometry>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief IMU preintegration factor plugin.
 *
 * Runs its own internal ISAM2 optimizer to estimate IMU biases using
 * lidar pose corrections (like LIO-SAM). Publishes high-rate IMU
 * odometry and TF from the IMU callback.
 */
class ImuIntegrationFactor : public FactorPlugin {
public:
  ImuIntegrationFactor() = default;
  ~ImuIntegrationFactor() override = default;

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
  // ---- Callbacks ----
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ---- IMU extrinsics ----
  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in);

  // ---- Internal ISAM2 helpers ----
  void resetOptimization();
  bool failureDetection(const gtsam::Vector3& vel,
                        const gtsam::imuBias::ConstantBias& bias);

  // ---- Subscription ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_incremental_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_fused_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- Transform fusion state ----
  Eigen::Isometry3d lidar_odom_affine_ = Eigen::Isometry3d::Identity();
  double lidar_odom_time_ = -1;
  std::deque<nav_msgs::msg::Odometry> imu_odom_queue_;

  // ---- IMU buffers (two queues like LIO-SAM) ----
  std::deque<sensor_msgs::msg::Imu> imu_queue_opt_;
  std::deque<sensor_msgs::msg::Imu> imu_queue_imu_;
  std::mutex imu_lock_;

  // ---- GTSAM preintegration (two integrators like LIO-SAM) ----
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_opt_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_imu_;

  // ---- Internal ISAM2 optimizer (for bias estimation) ----
  gtsam::ISAM2 optimizer_;
  gtsam::NonlinearFactorGraph graph_factors_;
  gtsam::Values graph_values_;

  // ---- Noise models ----
  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise2_;
  gtsam::Vector noise_between_bias_;

  // ---- State for internal optimizer ----
  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_ = gtsam::Vector3::Zero();
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  // State for real-time prediction
  gtsam::NavState prev_state_odom_;
  gtsam::imuBias::ConstantBias prev_bias_odom_;

  // IMU-base_link transforms (populated from TF in activate)
  gtsam::Pose3 imu_to_base_;
  gtsam::Pose3 base_to_imu_;

  // Internal optimizer key counter
  int key_ = 1;
  bool done_first_opt_ = false;
  double last_imu_t_imu_ = -1;
  double last_imu_t_opt_ = -1;
  bool initialized_ = false;
  bool active_ = false;

  // Latest IMU-predicted pose (for processFrame)
  gtsam::Pose3 latest_imu_pose_;
  bool has_imu_pose_ = false;
  std::mutex imu_pose_lock_;

  // ---- Parameters ----
  double acc_noise_ = 3.9939570888238808e-03;
  double gyr_noise_ = 1.5636343949698187e-03;
  double acc_bias_noise_ = 6.4356659353532566e-05;
  double gyr_bias_noise_ = 3.5640318696367613e-05;
  double gravity_ = 9.80511;
  double integration_noise_ = 1e-4;
  double prior_pose_sigma_ = 1e-2;
  double prior_vel_sigma_ = 1e4;
  double prior_bias_sigma_ = 1e-3;
  double correction_rot_sigma_ = 0.05;
  double correction_trans_sigma_ = 0.1;
  double correction_degrade_sigma_ = 1.0;
  double isam2_relinearize_threshold_ = 0.1;
  double max_velocity_ = 30.0;
  double max_bias_norm_ = 1.0;
  double default_imu_dt_ = 1.0 / 500.0;
  double quaternion_norm_threshold_ = 0.1;
  int graph_reset_interval_ = 100;

  // IMU-to-base_link extrinsic transform (looked up from TF: base_link_frame_ <- imu_frame_)
  tf2::Transform t_base_imu_;
  bool extrinsics_resolved_ = false;

  // Frame names
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";
  std::string imu_frame_ = "imu_link";
  std::string imu_odom_child_frame_ = "odom_imu";
};

}  // namespace eidos
