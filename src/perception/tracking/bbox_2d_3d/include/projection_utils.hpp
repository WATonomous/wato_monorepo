// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <optional>
#include <random>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ProjectionUtils
{
public:
  static void removeGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float distanceThreshold, int maxIterations);

  static std::optional<cv::Point2d> projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped & transform, const std::array<double, 12> & p, const pcl::PointXYZ & pt);

  // CLUSTERING FUNCTIONS
  // -----------------------------------------------------------------------------------------

  static void removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, int meanK, double stddevMulThresh);

  static void euclideanClusterExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double clusterTolerance,
    int minClusterSize,
    int maxClusterSize,
    const std::vector<pcl::PointIndices> & cluster_indices);

  static void assignClusterColors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & clustered_cloud);

  static void mergeClusters(
    const std::vector<pcl::PointIndices> & cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double mergeTolerance);

  static void filterClusterbyDensity(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    double densityWeight,
    double sizeWeight,
    double distanceWeight,
    double scoreThreshold);

  static bool computeClusterCentroid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices & cluster_indices,
    const pcl::PointXYZ & centroid);

  // ROI FUNCTIONS
  // ------------------------------------------------------------------------------------------------
  static double computeMaxIOU4Corners(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    const pcl::PointIndices & cluster_indices,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const vision_msgs::msg::Detection2DArray & detections,
    const float object_detection_confidence);

  static void computeHighestIOUCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const vision_msgs::msg::Detection2DArray & detections,
    const geometry_msgs::msg::TransformStamped & transform,
    const std::array<double, 12> & projection_matrix,
    const float object_detection_confidence);

  // BOUNDING BOX FUNCTIONS
  // ----------------------------------------------------------------------------------------

  static visualization_msgs::msg::MarkerArray computeBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const sensor_msgs::msg::PointCloud2 & msg);

private:
  static const int image_width_ = 1600;
  static const int image_height_ = 900;
};

#endif
