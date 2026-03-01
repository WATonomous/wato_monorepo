#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos {

/**
 * @brief GPS factor plugin.
 *
 * Subscribes to sensor_msgs/NavSatFix, converts to UTM internally,
 * manages the utm → map transform, and provides GPSFactor constraints
 * in map frame to the pose graph.
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
  std::vector<gtsam::NonlinearFactor::shared_ptr> getFactors(
      int state_index, const gtsam::Pose3& state_pose, double timestamp) override;
  void onOptimizationComplete(
      const gtsam::Values& optimized_values, bool loop_closure_detected) override;
  bool isReady() const override;
  std::string getReadyStatus() const override;

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void broadcastUtmToMap();

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  std::mutex gps_lock_;

  PointType last_gps_point_;
  bool has_last_gps_ = false;
  bool active_ = false;
  bool gps_received_ = false;

  // UTM → map transform (pure translation)
  Eigen::Vector3d utm_to_map_translation_ = Eigen::Vector3d::Zero();
  bool utm_to_map_initialized_ = false;
  int utm_zone_ = 0;
  bool utm_is_north_ = true;

  // Frame names
  std::string map_frame_ = "map";
  std::string utm_frame_ = "utm";

  // Parameters (populated from ROS params in onInitialize)
  float cov_threshold_;
  bool use_elevation_;
  float min_trajectory_length_;
  float gps_time_tolerance_;
  float min_gps_movement_;
  double min_noise_variance_;
};

}  // namespace eidos
