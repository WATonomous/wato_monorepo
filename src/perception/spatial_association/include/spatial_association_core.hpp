#ifndef SPATIAL_ASSOCIATION_CORE_HPP
#define SPATIAL_ASSOCIATION_CORE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include "utils/projection_utils.hpp"

#include <vector>
#include <memory>

// Core clustering logic without ROS dependencies
class SpatialAssociationCore {
 public:
  // Configuration structure for clustering parameters
  struct ClusteringParams {
    // Voxel downsampling
    float voxel_size = 0.2f;
    
    // Euclidean clustering
    double euclid_cluster_tolerance = 0.5;
    int euclid_min_cluster_size = 50;
    int euclid_max_cluster_size = 700;
    bool use_adaptive_clustering = true;
    double euclid_close_threshold = 10.0;
    double euclid_close_tolerance_mult = 1.5;
    
    // Density filtering
    double density_weight = 0.6;
    double size_weight = 0.8;
    double distance_weight = 0.7;
    double score_threshold = 0.6;
    
    // Merging
    double merge_threshold = 0.3;
    
    // Detection confidence
    float object_detection_confidence = 0.4f;
  };


  SpatialAssociationCore();
  ~SpatialAssociationCore() = default;

  // Set clustering parameters
  void setParams(const ClusteringParams& params);
  const ClusteringParams& getParams() const { return params_; }

  // Process point cloud: convert, downsample, and filter
  void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

  // Perform clustering on the filtered point cloud
  void performClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                       std::vector<pcl::PointIndices>& cluster_indices);

  // Compute cluster boxes from cluster indices
  std::vector<ProjectionUtils::Box3D> computeClusterBoxes(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
      const std::vector<pcl::PointIndices>& cluster_indices);

  // Assign colors to clusters for visualization
  void assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                          const std::vector<pcl::PointIndices>& cluster_indices,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cluster);

  // Compute cluster centroids
  void computeClusterCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                              const std::vector<pcl::PointIndices>& cluster_indices,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& centroid_cloud);

 private:
  ClusteringParams params_;
  
  // Working PCL objects for performance optimization
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_downsampled_cloud_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_colored_cluster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_centroid_cloud_;
};

#endif  // SPATIAL_ASSOCIATION_CORE_HPP

