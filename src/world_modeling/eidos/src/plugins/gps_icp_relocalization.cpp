#include "eidos/plugins/gps_icp_relocalization.hpp"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pluginlib/class_list_macros.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GpsIcpRelocalization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  // ---- Declare parameters ----
  node_->declare_parameter(prefix + ".gps_topic", "gps/odometry");
  node_->declare_parameter(prefix + ".gps_candidate_radius", 30.0);
  node_->declare_parameter(prefix + ".fitness_threshold", 0.3);
  node_->declare_parameter(prefix + ".max_icp_iterations", 100);
  node_->declare_parameter(prefix + ".submap_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".pointcloud_from", "lidar_kep_factor");
  node_->declare_parameter(prefix + ".gps_from", "gps_factor");

  // ---- Read parameters ----
  std::string gps_topic;
  node_->get_parameter(prefix + ".gps_topic", gps_topic);
  node_->get_parameter(prefix + ".gps_candidate_radius",
                       gps_candidate_radius_);
  node_->get_parameter(prefix + ".fitness_threshold", fitness_threshold_);
  node_->get_parameter(prefix + ".max_icp_iterations", max_icp_iterations_);
  node_->get_parameter(prefix + ".submap_leaf_size", submap_leaf_size_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".gps_from", gps_from_);

  // ---- Create GPS subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsIcpRelocalization::gpsCallback, this,
                std::placeholders::_1),
      sub_opts);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void GpsIcpRelocalization::activate() {
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void GpsIcpRelocalization::deactivate() {
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

// ---------------------------------------------------------------------------
// tryRelocalize - GPS + ICP relocalization against prior map
// ---------------------------------------------------------------------------
std::optional<RelocalizationResult> GpsIcpRelocalization::tryRelocalize(
    double /*timestamp*/) {
  if (!active_) return std::nullopt;

  // Need a prior map
  const auto& map_manager = core_->getMapManager();
  if (!map_manager.hasPriorMap()) return std::nullopt;

  // Get latest GPS position
  nav_msgs::msg::Odometry latest_gps;
  {
    std::lock_guard<std::mutex> lock(gps_lock_);
    if (gps_queue_.empty()) return std::nullopt;
    latest_gps = gps_queue_.back();
    gps_queue_.clear();
  }

  // Check GPS covariance
  float noise_x = static_cast<float>(latest_gps.pose.covariance[0]);
  float noise_y = static_cast<float>(latest_gps.pose.covariance[7]);
  if (noise_x > 5.0f || noise_y > 5.0f) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] GPS too noisy for relocalization (%.1f, %.1f)",
                 name_.c_str(), noise_x, noise_y);
    return std::nullopt;
  }

  float gps_x = static_cast<float>(latest_gps.pose.pose.position.x);
  float gps_y = static_cast<float>(latest_gps.pose.pose.position.y);
  float gps_z = static_cast<float>(latest_gps.pose.pose.position.z);

  // 1. Find candidate keyframes near GPS position
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  if (key_poses_3d->empty()) return std::nullopt;

  auto kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  kdtree->setInputCloud(key_poses_3d);

  PointType search_point;
  search_point.x = gps_x;
  search_point.y = gps_y;
  search_point.z = gps_z;

  std::vector<int> candidate_indices;
  std::vector<float> candidate_distances;
  kdtree->radiusSearch(search_point, gps_candidate_radius_, candidate_indices,
                       candidate_distances, 0);

  if (candidate_indices.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] No candidate keyframes near GPS position",
                 name_.c_str());
    return std::nullopt;
  }

  // 2. Assemble local submap from candidates using pointcloud_from plugin
  auto submap = pcl::make_shared<pcl::PointCloud<PointType>>();
  int best_candidate = candidate_indices[0];

  for (int idx : candidate_indices) {
    auto plugin_data = map_manager.getKeyframeDataForPlugin(idx, pointcloud_from_);
    if (plugin_data.empty()) continue;

    auto& pose = key_poses_6d->points[idx];
    Eigen::Affine3f t = poseTypeToAffine3f(pose);

    for (const auto& [data_key, data] : plugin_data) {
      try {
        auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
        if (cloud && !cloud->empty()) {
          pcl::PointCloud<PointType> transformed;
          pcl::transformPointCloud(*cloud, transformed, t);
          *submap += transformed;
        }
      } catch (const std::bad_any_cast&) {
        // Not a point cloud type, skip
      }
    }
  }

  if (submap->empty()) return std::nullopt;

  // Downsample submap
  pcl::VoxelGrid<PointType> downsample;
  downsample.setLeafSize(submap_leaf_size_, submap_leaf_size_, submap_leaf_size_);
  downsample.setInputCloud(submap);
  downsample.filter(*submap);

  // 3. Build source scan from the best candidate keyframe
  auto source_scan = pcl::make_shared<pcl::PointCloud<PointType>>();
  auto& best_pose = key_poses_6d->points[best_candidate];
  Eigen::Affine3f best_t = poseTypeToAffine3f(best_pose);

  auto source_data = map_manager.getKeyframeDataForPlugin(best_candidate, pointcloud_from_);
  for (const auto& [data_key, data] : source_data) {
    try {
      auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
      if (cloud && !cloud->empty()) {
        pcl::PointCloud<PointType> transformed;
        pcl::transformPointCloud(*cloud, transformed, best_t);
        *source_scan += transformed;
      }
    } catch (const std::bad_any_cast&) {
      // Not a point cloud type, skip
    }
  }

  if (source_scan->empty()) return std::nullopt;

  // 4. ICP alignment
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(gps_candidate_radius_);
  icp.setMaximumIterations(max_icp_iterations_);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(source_scan);
  icp.setInputTarget(submap);

  auto result_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  icp.align(*result_cloud);

  if (!icp.hasConverged()) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] ICP did not converge",
                 name_.c_str());
    return std::nullopt;
  }

  float fitness = static_cast<float>(icp.getFitnessScore());
  if (fitness > fitness_threshold_) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] ICP fitness too high: %.3f > %.3f", name_.c_str(),
                 fitness, fitness_threshold_);
    return std::nullopt;
  }

  // 5. Compute relocalized pose
  Eigen::Affine3f icp_correction;
  icp_correction = icp.getFinalTransformation();
  Eigen::Affine3f corrected = icp_correction * best_t;

  float rx, ry, rz, tx, ty, tz;
  pcl::getTranslationAndEulerAngles(corrected, tx, ty, tz, rx, ry, rz);
  gtsam::Pose3 relocalized_pose(gtsam::Rot3::RzRyRx(rx, ry, rz),
                                gtsam::Point3(tx, ty, tz));

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Relocalization succeeded! Fitness: %.3f, "
              "Matched keyframe: %d, Pose: (%.1f, %.1f, %.1f)",
              name_.c_str(), fitness, best_candidate, tx, ty, tz);

  RelocalizationResult result;
  result.pose = relocalized_pose;
  result.fitness_score = fitness;
  result.matched_keyframe_index = best_candidate;
  return result;
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void GpsIcpRelocalization::gpsCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsIcpRelocalization,
                       eidos::RelocalizationPlugin)
