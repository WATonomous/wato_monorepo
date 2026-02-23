#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "eidos/factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief GPS factor plugin.
 *
 * Subscribes to GPS odometry, provides GPSFactor constraints
 * to the pose graph when GPS is available and reliable.
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

private:
  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;

  std::deque<nav_msgs::msg::Odometry> gps_queue_;
  std::mutex gps_lock_;

  PointType last_gps_point_;
  bool has_last_gps_ = false;
  bool active_ = false;

  // Parameters
  float cov_threshold_ = 2.0;
  bool use_elevation_ = false;
  float min_trajectory_length_ = 5.0;   // meters before injecting GPS
  float gps_time_tolerance_ = 0.2;      // seconds to match GPS to state
  float min_gps_movement_ = 5.0;        // meters between GPS injections
  float no_elevation_noise_ = 0.01;     // z noise when elevation disabled
  double min_noise_variance_ = 1.0;     // floor for GPS noise variance
};

}  // namespace eidos
