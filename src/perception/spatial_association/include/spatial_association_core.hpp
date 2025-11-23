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
    
    // Quality filtering parameters (improved physics-based filtering)
    double max_distance = 60.0;
    int min_points = 5;
    float min_height = 0.15f;
    int min_points_default = 10;
    int min_points_far = 8;
    int min_points_medium = 12;
    int min_points_large = 30;
    double distance_threshold_far = 30.0;
    double distance_threshold_medium = 20.0;
    float volume_threshold_large = 8.0f;
    float min_density = 5.0f;
    float max_density = 1000.0f;
    float max_dimension = 15.0f;
    float max_aspect_ratio = 15.0f;
  };


  /**
   * @brief Constructor - initializes working PCL objects and voxel filter
   */
  SpatialAssociationCore();
  
  /**
   * @brief Destructor
   */
  ~SpatialAssociationCore() = default;

  /**
   * @brief Sets clustering parameters and updates voxel filter
   * @param params Clustering parameters structure
   */
  void setParams(const ClusteringParams& params);
  
  /**
   * @brief Gets current clustering parameters
   * @return Reference to current clustering parameters
   */
  const ClusteringParams& getParams() const { return params_; }

  /**
   * @brief Processes point cloud by applying voxel downsampling
   * @param input_cloud Input point cloud
   * @param output_cloud Output downsampled point cloud
   */
  void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

  /**
   * @brief Performs clustering on the filtered point cloud
   * @param filtered_cloud Input filtered point cloud (modified in place)
   * @param cluster_indices Output vector of cluster indices
   * @details Performs Euclidean clustering, density filtering, and cluster merging
   */
  void performClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                       std::vector<pcl::PointIndices>& cluster_indices);

  /**
   * @brief Computes 3D bounding boxes for clusters
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @return Vector of 3D bounding boxes
   */
  std::vector<ProjectionUtils::Box3D> computeClusterBoxes(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
      const std::vector<pcl::PointIndices>& cluster_indices) const;

  /**
   * @brief Assigns random colors to clusters for visualization
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @param colored_cluster Output colored point cloud
   */
  void assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                          const std::vector<pcl::PointIndices>& cluster_indices,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cluster) const;

  /**
   * @brief Computes centroids for all clusters
   * @param filtered_cloud Input filtered point cloud
   * @param cluster_indices Vector of cluster indices
   * @param centroid_cloud Output point cloud containing cluster centroids
   */
  void computeClusterCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                              const std::vector<pcl::PointIndices>& cluster_indices,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& centroid_cloud) const;

 private:
  ClusteringParams params_;
  
  // Working PCL objects for performance optimization
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_colored_cluster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_centroid_cloud_;
};

#endif  // SPATIAL_ASSOCIATION_CORE_HPP

