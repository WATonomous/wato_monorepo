#ifndef WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_
#define WORLD_MODELING_HD_MAP_LOCALIZATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <GeographicLib/LocalCartesian.hpp>

// GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

class LocalizationNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  LocalizationNode();

private:
  void setupReference(const sensor_msgs::msg::NavSatFix::SharedPtr ref_point);
  gtsam::Point3 convertGPSToENU(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void optimize();

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Sample object that handles data processing and validation
  world_modeling::hd_map::Sample sample_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::noiseModel::Diagonal::shared_ptr gps_noise_model_;
  GeographicLib::LocalCartesian enu_;

  int odom_counter_;
  int gps_counter_;
};

#endif // LOCALIZATION_NODE_HPP_