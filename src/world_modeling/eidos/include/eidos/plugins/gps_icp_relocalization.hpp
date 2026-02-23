#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "eidos/relocalization_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

/**
 * @brief GPS + ICP relocalization plugin.
 *
 * Uses GPS to find candidate keyframes in the prior map, assembles
 * a local submap, and performs ICP alignment to relocalize.
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
  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;

  std::deque<nav_msgs::msg::Odometry> gps_queue_;
  std::mutex gps_lock_;

  bool active_ = false;

  // Parameters
  float gps_candidate_radius_ = 30.0;
  float fitness_threshold_ = 0.3;
  int max_icp_iterations_ = 100;
  float submap_leaf_size_ = 0.4;
  std::string pointcloud_from_ = "lidar_kep_factor";
  std::string gps_from_ = "gps_factor";
};

}  // namespace eidos
