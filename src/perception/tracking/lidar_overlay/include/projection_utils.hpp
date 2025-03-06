#ifndef PROJECTION_UTILS_HPP
#define PROJECTION_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <optional>
#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ProjectionUtils {
public:
    ProjectionUtils();

    static void removeGroundPlane(PointCloud::Ptr& cloud);

    static std::optional<cv::Point2d> projectLidarToCamera(
        const geometry_msgs::msg::TransformStamped& transform,
        const std::array<double, 12>& p,
        const pcl::PointXYZ& pt,
        int image_width = 1600,
        int image_height = 900);

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


    // ROI FUNCTIONS ------------------------------------------------------------------------------------------------

private:
};

#endif