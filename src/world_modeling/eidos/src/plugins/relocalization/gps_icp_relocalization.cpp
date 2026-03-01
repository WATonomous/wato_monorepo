#include "eidos/plugins/relocalization/gps_icp_relocalization.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/points/point_cloud.hpp>

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
  node_->declare_parameter(prefix + ".gps_topic", "gps/fix");
  node_->declare_parameter(prefix + ".gps_candidate_radius", 30.0);
  node_->declare_parameter(prefix + ".fitness_threshold", 0.3);
  node_->declare_parameter(prefix + ".max_icp_iterations", 100);
  node_->declare_parameter(prefix + ".submap_leaf_size", 0.4);
  node_->declare_parameter(prefix + ".max_correspondence_distance", 2.0);
  node_->declare_parameter(prefix + ".num_threads", 4);
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
  node_->get_parameter(prefix + ".max_correspondence_distance",
                       max_correspondence_distance_);
  node_->get_parameter(prefix + ".num_threads", num_threads_);
  node_->get_parameter(prefix + ".pointcloud_from", pointcloud_from_);
  node_->get_parameter(prefix + ".gps_from", gps_from_);

  // ---- Create GPS subscription ----
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GpsIcpRelocalization::gpsCallback, this,
                std::placeholders::_1),
      sub_opts);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized (NavSatFix mode)", name_.c_str());
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
// Helpers: convert between PCL and small_gicp point clouds
// ---------------------------------------------------------------------------
namespace {

std::shared_ptr<small_gicp::PointCloud> pclToSmallGicp(
    const pcl::PointCloud<PointType>::Ptr& pcl_cloud) {
  auto cloud = std::make_shared<small_gicp::PointCloud>();
  cloud->resize(pcl_cloud->size());
  for (size_t i = 0; i < pcl_cloud->size(); i++) {
    cloud->point(i) = Eigen::Vector4d(
        pcl_cloud->points[i].x, pcl_cloud->points[i].y,
        pcl_cloud->points[i].z, 1.0);
  }
  return cloud;
}

}  // namespace

// ---------------------------------------------------------------------------
// tryRelocalize - GPS + ICP relocalization against prior map
// ---------------------------------------------------------------------------
std::optional<RelocalizationResult> GpsIcpRelocalization::tryRelocalize(
    double /*timestamp*/) {
  if (!active_) return std::nullopt;

  // Need a prior map
  const auto& map_manager = core_->getMapManager();
  if (!map_manager.hasPriorMap()) return std::nullopt;

  // Get latest GPS fix
  sensor_msgs::msg::NavSatFix latest_fix;
  {
    std::lock_guard<std::mutex> lock(gps_lock_);
    if (gps_queue_.empty()) return std::nullopt;
    latest_fix = gps_queue_.back();
    gps_queue_.clear();
  }

  // Filter by status
  if (latest_fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return std::nullopt;
  }

  // Check covariance
  double noise_x = latest_fix.position_covariance[0];
  double noise_y = latest_fix.position_covariance[4];
  if (noise_x > 5.0 || noise_y > 5.0) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] GPS too noisy for relocalization (%.1f, %.1f)",
                 name_.c_str(), noise_x, noise_y);
    return std::nullopt;
  }

  // Convert to UTM
  UtmCoordinate utm = latLonToUtm(
      latest_fix.latitude, latest_fix.longitude, latest_fix.altitude);

  // Load saved utm_to_map offset from MapManager global data
  auto global_offset = map_manager.getGlobalData("gps_factor/utm_to_map");
  if (!global_offset.has_value()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] No utm_to_map offset available yet", name_.c_str());
    return std::nullopt;
  }

  auto offset_vec = std::any_cast<Eigen::Vector4d>(global_offset.value());
  Eigen::Vector3d utm_to_map = offset_vec.head<3>();

  // Compute approximate map-frame position
  Eigen::Vector3d map_guess(
      utm.easting - utm_to_map.x(),
      utm.northing - utm_to_map.y(),
      utm.altitude - utm_to_map.z());

  // 1. Find candidate keyframes near GPS position in map frame
  auto key_poses_3d = map_manager.getKeyPoses3D();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  if (key_poses_3d->empty()) return std::nullopt;

  auto kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();
  kdtree->setInputCloud(key_poses_3d);

  PointType search_point;
  search_point.x = static_cast<float>(map_guess.x());
  search_point.y = static_cast<float>(map_guess.y());
  search_point.z = static_cast<float>(map_guess.z());

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

  // 2. Assemble local submap from candidates
  auto submap_pcl = pcl::make_shared<pcl::PointCloud<PointType>>();
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
          *submap_pcl += transformed;
        }
      } catch (const std::bad_any_cast&) {
        continue;
      }
    }
  }

  if (submap_pcl->empty()) return std::nullopt;

  // Downsample submap
  pcl::VoxelGrid<PointType> downsample;
  downsample.setLeafSize(submap_leaf_size_, submap_leaf_size_, submap_leaf_size_);
  downsample.setInputCloud(submap_pcl);
  downsample.filter(*submap_pcl);

  // 3. Build source scan from the best candidate keyframe
  auto source_pcl = pcl::make_shared<pcl::PointCloud<PointType>>();
  auto& best_pose = key_poses_6d->points[best_candidate];
  Eigen::Affine3f best_t = poseTypeToAffine3f(best_pose);

  auto source_data = map_manager.getKeyframeDataForPlugin(best_candidate, pointcloud_from_);
  for (const auto& [data_key, data] : source_data) {
    try {
      auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
      if (cloud && !cloud->empty()) {
        pcl::PointCloud<PointType> transformed;
        pcl::transformPointCloud(*cloud, transformed, best_t);
        *source_pcl += transformed;
      }
    } catch (const std::bad_any_cast&) {
      continue;
    }
  }

  if (source_pcl->empty()) return std::nullopt;

  // 4. ICP alignment using small_gicp
  auto source_gicp = pclToSmallGicp(source_pcl);
  auto target_gicp = pclToSmallGicp(submap_pcl);

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::ICP;
  setting.max_correspondence_distance = max_correspondence_distance_;
  setting.num_threads = num_threads_;
  setting.downsampling_resolution = submap_leaf_size_;
  setting.max_iterations = max_icp_iterations_;

  auto result = small_gicp::align(
      target_gicp->points, source_gicp->points, Eigen::Isometry3d::Identity(), setting);

  if (!result.converged) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] ICP did not converge",
                 name_.c_str());
    return std::nullopt;
  }

  double fitness = result.error;
  if (fitness > fitness_threshold_) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[%s] ICP fitness too high: %.3f > %.3f", name_.c_str(),
                 fitness, fitness_threshold_);
    return std::nullopt;
  }

  // 5. Compute relocalized pose
  Eigen::Affine3f icp_correction;
  icp_correction.matrix() = result.T_target_source.matrix().cast<float>();
  Eigen::Affine3f corrected = icp_correction * best_t;

  float rx, ry, rz, tx, ty, tz;
  pcl::getTranslationAndEulerAngles(corrected, tx, ty, tz, rx, ry, rz);
  gtsam::Pose3 relocalized_pose(gtsam::Rot3::RzRyRx(rx, ry, rz),
                                gtsam::Point3(tx, ty, tz));

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Relocalization succeeded! Fitness: %.3f, "
              "Matched keyframe: %d, Pose: (%.1f, %.1f, %.1f)",
              name_.c_str(), fitness, best_candidate, tx, ty, tz);

  // Refine utm → map offset using the precise ICP result
  Eigen::Vector3d utm_pos(utm.easting, utm.northing, utm.altitude);
  Eigen::Vector3d refined_offset = utm_pos - Eigen::Vector3d(tx, ty, tz);
  double zone_encoded = static_cast<double>(utm.zone * 10 + (utm.is_north ? 1 : 0));
  Eigen::Vector4d refined_global(refined_offset.x(), refined_offset.y(),
                                  refined_offset.z(), zone_encoded);
  core_->getMapManager().setGlobalData("gps_factor/utm_to_map", refined_global);

  RelocalizationResult reloc_result;
  reloc_result.pose = relocalized_pose;
  reloc_result.fitness_score = fitness;
  reloc_result.matched_keyframe_index = best_candidate;
  return reloc_result;
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void GpsIcpRelocalization::gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(gps_lock_);
  gps_queue_.push_back(*msg);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::GpsIcpRelocalization,
                       eidos::RelocalizationPlugin)
