#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>

#include <pcl/point_cloud.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/registration/registration_result.hpp>

#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/types.hpp"

namespace eidos {

class LisoFactor : public FactorPlugin {
public:
  LisoFactor() = default;
  ~LisoFactor() override = default;

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
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rebuildSubmap();
  std::vector<gtsam::Key> bfsCollectStates(gtsam::Key start, double radius);

  // State
  std::atomic<bool> first_scan_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> scan_received_{false};

  // Cached submap (read by lidarCallback, written by rebuildSubmap)
  small_gicp::PointCloud::Ptr cached_submap_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> cached_submap_tree_;
  mutable std::shared_mutex submap_mtx_;

  // Cached GICP result for getFactors
  small_gicp::RegistrationResult cached_result_;
  gtsam::Pose3 cached_pose_;
  small_gicp::PointCloud::Ptr cached_cloud_;  // body-frame (for submap assembly)
  pcl::PointCloud<PointType>::Ptr cached_pcl_cloud_;  // body-frame (for MapManager/visualizers)
  double cached_timestamp_ = 0.0;
  bool has_cached_result_ = false;
  std::mutex result_mtx_;

  // Subscription + publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Last successful match tracking (for initial guess computation)
  gtsam::Pose3 last_matched_pose_;       // GICP result at last successful match
  gtsam::Pose3 last_matched_mm_pose_;    // motion model pose at last successful match time
  bool has_last_match_ = false;

  // Distance gating + relative factor tracking
  gtsam::Point3 last_factor_position_ = gtsam::Point3(0, 0, 0);
  bool has_last_factor_ = false;
  gtsam::Key prev_liso_key_ = 0;
  gtsam::Pose3 prev_liso_pose_;
  bool has_prev_liso_ = false;

  // Config
  double scan_ds_resolution_ = 0.5;
  double submap_ds_resolution_ = 0.5;
  double submap_radius_ = 50.0;
  double max_correspondence_distance_ = 2.0;
  int max_iterations_ = 20;
  int num_threads_ = 4;
  int num_neighbors_ = 10;
  int max_submap_states_ = 25;
  int min_inliers_ = 50;
  double min_noise_ = 0.01;
  double min_scan_distance_ = 1.0;
  std::string lidar_frame_ = "velodyne";
  std::string map_frame_ = "map";
  std::string base_link_frame_ = "base_footprint";
  std::vector<double> odom_pose_cov_ = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
  Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
  bool has_tf_ = false;
};

}  // namespace eidos
