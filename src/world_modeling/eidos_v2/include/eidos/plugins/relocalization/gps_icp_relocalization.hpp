#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>

#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/utils/types.hpp"
#include "eidos/utils/utm.hpp"

namespace eidos {

/**
 * @brief GPS + ICP relocalization against a prior map.
 *
 * Subscribes to live GPS, LiDAR, and IMU. Uses GPS to find candidate
 * keyframes in the prior map, assembles a world-frame submap from their
 * clouds, then aligns the live LiDAR scan against that submap via GICP.
 * The init_guess comes from GPS position + IMU gravity + nearest keyframe yaw.
 */
class GpsIcpRelocalization : public RelocalizationPlugin {
public:
  GpsIcpRelocalization() = default;
  ~GpsIcpRelocalization() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  std::optional<RelocalizationResult> tryRelocalize(double timestamp) override;

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Buffered sensor data
  std::deque<sensor_msgs::msg::NavSatFix> gps_queue_;
  std::mutex gps_lock_;

  small_gicp::PointCloud::Ptr latest_scan_;
  std::mutex scan_lock_;

  double latest_imu_roll_ = 0.0;
  double latest_imu_pitch_ = 0.0;
  std::mutex imu_lock_;
  bool has_imu_ = false;

  bool active_ = false;

  // Parameters
  double gps_candidate_radius_ = 30.0;
  double fitness_threshold_ = 0.3;
  int max_iterations_ = 100;
  double scan_ds_resolution_ = 0.5;
  double submap_leaf_size_ = 0.4;
  double max_correspondence_distance_ = 2.0;
  int num_threads_ = 4;
  int num_neighbors_ = 10;
  std::string pointcloud_from_;
  std::string gps_from_;
  std::string lidar_frame_;
};

}  // namespace eidos
