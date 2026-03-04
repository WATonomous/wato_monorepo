#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point3.h>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos {

/**
 * @brief GPS factor plugin.
 *
 * Subscribes to sensor_msgs/NavSatFix, converts to UTM internally,
 * manages the utm → map transform via a jointly-optimized bias variable,
 * and provides BiasedGPSFactor constraints to the pose graph.
 *
 * The bias (Point3) represents the utm→map offset and is refined by the
 * optimizer as GPS data accumulates. State 0 must be anchored with a tight
 * translation prior so that the bias is well-determined.
 */
class GpsFactor : public FactorPlugin {
public:
  GpsFactor() = default;
  ~GpsFactor() override = default;

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

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void broadcastUtmToMap();

  /// Rotate a UTM position into the map frame using the initial heading
  Eigen::Vector3d utmToMap(const Eigen::Vector3d& utm_pos) const;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  std::mutex gps_lock_;

  PointType last_gps_point_;
  bool has_last_gps_ = false;
  bool active_ = false;
  bool gps_received_ = false;

  // IMU heading — used to align map frame with ENU/UTM
  std::mutex imu_orientation_lock_;
  double latest_imu_yaw_ = 0.0;
  std::atomic<bool> has_imu_orientation_{false};
  double initial_yaw_ = 0.0;                           // yaw captured at bias init (ENU convention)
  Eigen::Matrix3d R_map_enu_ = Eigen::Matrix3d::Identity();  // rotation from ENU/UTM to map frame

  // Bias variable for joint optimization: bias = rotated_utm_pos - map_pos
  gtsam::Key bias_key_ = gtsam::Symbol('g', 0);
  bool bias_initialized_ = false;
  gtsam::Point3 latest_bias_ = gtsam::Point3(0, 0, 0);

  // UTM state
  int utm_zone_ = 0;
  bool utm_is_north_ = true;

  // Frame names
  std::string map_frame_ = "map";
  std::string utm_frame_ = "utm";

  // Parameters (populated from ROS params in onInitialize)
  float cov_threshold_;
  bool use_elevation_;
  float min_gps_movement_;
  std::vector<double> bias_prior_cov_;  // [x, y, z]
  std::vector<double> gps_cov_;         // [x, y, z] BiasedGPSFactor measurement covariance
};

}  // namespace eidos
