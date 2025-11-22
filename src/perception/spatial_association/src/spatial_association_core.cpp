#include "spatial_association_core.hpp"

SpatialAssociationCore::SpatialAssociationCore() {
  // Initialize working PCL objects for performance optimization
  working_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_downsampled_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Initialize voxel filter with default size
  voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);
}

void SpatialAssociationCore::setParams(const ClusteringParams& params) {
  params_ = params;
  voxel_filter_.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
}

void SpatialAssociationCore::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
  if (!input_cloud || input_cloud->empty()) {
    output_cloud->clear();
    return;
  }

  // Convert to working object
  *working_cloud_ = *input_cloud;

  // Apply downsampling using working objects
  voxel_filter_.setInputCloud(working_cloud_);
  voxel_filter_.filter(*working_downsampled_cloud_);

  output_cloud = working_downsampled_cloud_;
}

void SpatialAssociationCore::performClustering(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    std::vector<pcl::PointIndices>& cluster_indices) {
  // Clear previous clusters
  cluster_indices.clear();

  if (!filtered_cloud || filtered_cloud->empty()) {
    return;
  }

  // CLUSTERING-----------------------------------------------------------------------------------------------
  // Perform Euclidean clustering, populate cluster_indices
  if (params_.use_adaptive_clustering) {
    // Use adaptive clustering: larger tolerance for closer points to prevent fragmentation
    ProjectionUtils::adaptiveEuclideanClusterExtraction(
        filtered_cloud,
        params_.euclid_cluster_tolerance,
        params_.euclid_min_cluster_size,
        params_.euclid_max_cluster_size,
        cluster_indices,
        params_.euclid_close_threshold,
        params_.euclid_close_tolerance_mult);
  } else {
    // Use fixed cluster tolerance (original behavior)
    ProjectionUtils::euclideanClusterExtraction(
        filtered_cloud,
        params_.euclid_cluster_tolerance,
        params_.euclid_min_cluster_size,
        params_.euclid_max_cluster_size,
        cluster_indices);
  }

  // Precompute cluster stats once to avoid redundant point scans
  auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_cloud, cluster_indices);

  // Filter clusters by density, size and distance
  ProjectionUtils::filterClusterbyDensity(
      cluster_stats,
      cluster_indices,
      params_.density_weight,
      params_.size_weight,
      params_.distance_weight,
      params_.score_threshold);

  // Merge clusters that are close to each other, determined through distance between their centroids
  ProjectionUtils::mergeClusters(
      cluster_indices,
      filtered_cloud,
      cluster_stats,
      params_.merge_threshold);
}

std::vector<ProjectionUtils::Box3D> SpatialAssociationCore::computeClusterBoxes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {
  return ProjectionUtils::computeClusterBoxes(filtered_cloud, cluster_indices);
}

void SpatialAssociationCore::assignClusterColors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cluster) {
  ProjectionUtils::assignClusterColors(filtered_cloud, cluster_indices, colored_cluster);
}

void SpatialAssociationCore::computeClusterCentroids(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& centroid_cloud) {
  centroid_cloud->clear(); // Clear previous data
  for (const auto& ci : cluster_indices) {
    pcl::PointXYZ c;
    ProjectionUtils::computeClusterCentroid(filtered_cloud, ci, c);
    centroid_cloud->points.push_back(c);
  }
  centroid_cloud->width = centroid_cloud->points.size();
  centroid_cloud->height = 1;
  centroid_cloud->is_dense = true;
}

