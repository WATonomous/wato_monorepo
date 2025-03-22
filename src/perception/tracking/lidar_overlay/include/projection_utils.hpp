#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>
#include <array>
#include <optional>
#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ProjectionUtils {
public:

    static void removeGroundPlane(PointCloud::Ptr& cloud, float distanceThreshold, int maxIterations);

    static std::optional<cv::Point2d> projectLidarToCamera(
        const geometry_msgs::msg::TransformStamped& transform,
        const std::array<double, 12>& p,
        const pcl::PointXYZ& pt);

    // CLUSTERING FUNCTIONS -----------------------------------------------------------------------------------------

    static void removeOutliers(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int meanK,
        double stddevMulThresh);

    static void dbscanCluster(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double clusterTolerance,
        int minClusterSize,
        int maxClusterSize,
        std::vector<pcl::PointIndices>& cluster_indices);

    static void assignClusterColors(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud);

    static void mergeClusters(
        std::vector<pcl::PointIndices>& cluster_indices,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double mergeTolerance);

    static void filterClusterbyDensity(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
        double densityWeight,
        double sizeWeight,
        double distanceWeight,
        double scoreThreshold);

    static bool computeClusterCentroid(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointIndices& cluster_indices,
        pcl::PointXYZ& centroid);

    // ROI FUNCTIONS ------------------------------------------------------------------------------------------------
    static bool pointIn2DBoundingBox(
        const pcl::PointXYZ& point,
        const vision_msgs::msg::Detection2DArray& detections,
        const geometry_msgs::msg::TransformStamped& transform,
        const std::array<double, 12>& projection_matrix);

    static void filterClusterByBoundingBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        std::vector<pcl::PointIndices>& cluster_indices,
        const vision_msgs::msg::Detection2DArray& detections,
        const geometry_msgs::msg::TransformStamped& transform,
        const std::array<double, 12>& projection_matrix);

    // BOUNDING BOX FUNCTIONS ----------------------------------------------------------------------------------------
    static visualization_msgs::msg::MarkerArray computeBoundingBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
        const sensor_msgs::msg::PointCloud2& msg);

    
private:
    static const int image_width_ = 1600;
    static const int image_height_ = 900;
};

#endif