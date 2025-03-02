#include "projection_utils.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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