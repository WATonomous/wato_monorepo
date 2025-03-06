#include "projection_utils.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <random>

void ProjectionUtils::removeGroundPlane(PointCloud::Ptr& cloud) {
    if (cloud->empty()) {
        return;
    }

    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    ransac.setDistanceThreshold(0.3);
    ransac.setMaxIterations(2000);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    inliers_ptr->indices = inliers;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_ptr);
    extract.setNegative(true);

    PointCloud::Ptr cloud_filtered(new PointCloud());
    extract.filter(*cloud_filtered);

    cloud->swap(*cloud_filtered);
}

std::optional<cv::Point2d> ProjectionUtils::projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& p,
    const pcl::PointXYZ& pt,
    int image_width,
    int image_height) {
    // Convert LiDAR point to geometry_msgs::Point
    geometry_msgs::msg::Point orig_pt;
    orig_pt.x = pt.x;
    orig_pt.y = pt.y;
    orig_pt.z = pt.z;

    // Transform LiDAR point to camera frame
    geometry_msgs::msg::Point trans_pt;
    tf2::doTransform(orig_pt, trans_pt, transform);

    // Reject points behind the camera (z < 1)
    if (trans_pt.z < 1) {
        return std::nullopt;
    }

    // Project the point onto the image plane using the camera projection matrix
    double u = p[0] * trans_pt.x + p[1] * trans_pt.y + p[2] * trans_pt.z + p[3];
    double v = p[4] * trans_pt.x + p[5] * trans_pt.y + p[6] * trans_pt.z + p[7];
    double w = p[8] * trans_pt.x + p[9] * trans_pt.y + p[10] * trans_pt.z + p[11];

    // Normalize the projected coordinates
    cv::Point2d proj_pt;
    proj_pt.x = u / w;
    proj_pt.y = v / w;

    // Check if the projected point is within the image bounds
    if (proj_pt.x >= 0 && proj_pt.x < image_width && proj_pt.y >= 0 && proj_pt.y < image_height) {
        return proj_pt;
    }

    // Return nullopt if the point is outside the image bounds
    return std::nullopt;
}

// CLUSTERING FUNCTIONS ------------------------------------------------------------------------------------------------

void ProjectionUtils::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int meanK, double stddevMulThresh) {
    if (cloud->empty()) return; // Handle empty cloud

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*cloud);
}

void ProjectionUtils::dbscanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    double clusterTolerance,
                                    int minClusterSize,
                                    int maxClusterSize,
                                    std::vector<pcl::PointIndices>& cluster_indices) {
    if (cloud->empty()) return; // Handle empty cloud

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void ProjectionUtils::assignClusterColors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                            const std::vector<pcl::PointIndices>& cluster_indices,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud) {
    if (cloud->empty() || cluster_indices.empty()) return; // Handle empty cloud or clusters

    clustered_cloud->clear(); // Clear previous data
    clustered_cloud->points.reserve(cloud->size()); // Reserve memory for efficiency

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (const auto& indices : cluster_indices) {
        int r = dis(gen);
        int g = dis(gen);
        int b = dis(gen);
        for (const auto& index : indices.indices) {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = r;
            point.g = g;
            point.b = b;
            clustered_cloud->points.push_back(point);
        }
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;
}

// ROI FUNCTIONS ------------------------------------------------------------------------------------------------
