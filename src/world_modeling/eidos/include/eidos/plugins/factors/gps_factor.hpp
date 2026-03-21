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
 * @brief GPS factor plugin (latching, unary GPSFactor).
 *
 * Subscribes to NavSatFix, converts to UTM internally, and provides
 * unary GPSFactor constraints to the pose graph via latchFactors().
 *
 * GPS never creates its own states. When a new state is created by another
 * plugin (e.g. LISO), SlamCore calls latchFactors() and GPS attaches a
 * unary GPSFactor if it has a valid fix.
 *
 * The UTM→map offset is computed once at initialization using the IMU heading
 * and the current SLAM pose. It is NOT optimized by ISAM2 (no shared bias
 * variable), avoiding Bayes tree hub effects. The utm→map TF is broadcast
 * as a static transform.
 */
class GpsFactor : public FactorPlugin {
public:
  GpsFactor() = default;
  ~GpsFactor() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  // GPS never creates states — these are no-ops
  std::optional<gtsam::Pose3> processFrame(double) override { return std::nullopt; }
  StampedFactorResult getFactors(gtsam::Key) override { return {}; }
  bool hasData() const override { return false; }

  // GPS latches onto existing states
  StampedFactorResult latchFactors(gtsam::Key key, double timestamp) override;

  // Always ready (latching-only plugin, never blocks SLAM startup)
  bool isReady() const override;
  std::string getReadyStatus() const override;

  void saveData(const std::string& plugin_dir) override;
  void loadData(const std::string& plugin_dir) override;

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void broadcastUtmToMap();

  /// Rotate a UTM position into the map frame using the initial heading
  Eigen::Vector3d utmToMap(const Eigen::Vector3d& utm_pos) const;

  // Subscriptions + publishers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // GPS queue
  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  mutable std::mutex gps_lock_;

  // Distance gating state
  PointType last_gps_point_;
  bool has_last_gps_ = false;
  bool active_ = false;
  bool gps_received_ = false;

  // IMU heading — used to align map frame with ENU/UTM
  std::mutex imu_orientation_lock_;
  double latest_imu_yaw_ = 0.0;
  std::atomic<bool> has_imu_orientation_{false};
  double initial_yaw_ = 0.0;
  Eigen::Matrix3d R_map_enu_ = Eigen::Matrix3d::Identity();

  // Fixed UTM→map offset (computed once, not optimized by ISAM2)
  bool offset_initialized_ = false;
  gtsam::Point3 utm_to_map_offset_{0, 0, 0};  // offset = R_map_enu * utm_pos - map_pos

  // UTM state
  int utm_zone_ = 0;
  bool utm_is_north_ = true;

  // Frame names
  std::string map_frame_ = "map";
  std::string utm_frame_ = "utm";

  // Parameters
  float max_cov_;
  float min_radius_;
  bool use_elevation_;
  std::vector<double> gps_cov_;
  double pose_cov_threshold_;   // skip GPS when ISAM2 pose covariance x,y < this
  bool add_factors_ = true;     // whether to add GPSFactor to the graph (false = visualization only)
};

}  // namespace eidos
