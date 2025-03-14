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
    if (cloud->empty()) return;

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
    const pcl::PointXYZ& pt) {

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
    if (proj_pt.x >= 0 && proj_pt.x < image_width_ && proj_pt.y >= 0 && proj_pt.y < image_height_) {
        return proj_pt;
    }

    // Return nullopt if the point is outside the image bounds
    return std::nullopt;
}

std::optional<cv::Point3d> ProjectionUtils::projectLidarToCamera3D(
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& p,
    const pcl::PointXYZ& pt) {

    geometry_msgs::msg::Point orig_pt;
    orig_pt.x = pt.x;
    orig_pt.y = pt.y;
    orig_pt.z = pt.z;

    geometry_msgs::msg::Point trans_pt;
    tf2::doTransform(orig_pt, trans_pt, transform);

    if (trans_pt.z < 1) {
        return std::nullopt;
    }

    double u = p[0] * trans_pt.x + p[1] * trans_pt.y + p[2] * trans_pt.z + p[3];
    double v = p[4] * trans_pt.x + p[5] * trans_pt.y + p[6] * trans_pt.z + p[7];
    double w = p[8] * trans_pt.x + p[9] * trans_pt.y + p[10] * trans_pt.z + p[11];

    cv::Point3d proj_pt(u / w, v / w, trans_pt.z); // Include depth
    return proj_pt;
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
    
   /*  RCLCPP_INFO(rclcpp::get_logger("ProjectionUtils"), "[DBSCAN] Cloud contains %ld points.", cloud->size());
    if (cloud->empty()) return; // Handle empty cloud */

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
    clustered_cloud->header = cloud->header;
}


void ProjectionUtils::mergeClusters(std::vector<pcl::PointIndices>& cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double mergeTolerance) {
    if (cloud->empty() || cluster_indices.empty()) return;

    // Vector to store whether a cluster has been merged
    std::vector<bool> merged(cluster_indices.size(), false);

    // Iterate through all pairs of clusters
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (merged[i]) continue; // Skip if already merged

        // Compute centroid of cluster i
        Eigen::Vector4f centroid_i;
        pcl::compute3DCentroid(*cloud, cluster_indices[i].indices, centroid_i);

        for (size_t j = i + 1; j < cluster_indices.size(); ++j) {
            if (merged[j]) continue; // Skip if already merged

            // Compute centroid of cluster j
            Eigen::Vector4f centroid_j;
            pcl::compute3DCentroid(*cloud, cluster_indices[j].indices, centroid_j);

            // Calculate Euclidean distance between centroids
                double distance = (centroid_i - centroid_j).norm();

            // Merge clusters if distance is below the threshold
            if (distance < mergeTolerance) {
                // Merge cluster j into cluster i
                cluster_indices[i].indices.insert(cluster_indices[i].indices.end(),
                                cluster_indices[j].indices.begin(),
                                cluster_indices[j].indices.end());
                merged[j] = true; // Mark cluster j as merged
            }
        }
    }

    // Remove merged clusters from the list
    std::vector<pcl::PointIndices> filtered_clusters;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (!merged[i]) {
            filtered_clusters.push_back(cluster_indices[i]);
        }
    }
    cluster_indices = filtered_clusters;
}


bool ProjectionUtils::computeClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices& cluster_indices,
    pcl::PointXYZ& centroid) {

    if (cloud->empty() || cluster_indices.indices.empty()) return false; 

    // Compute centroid of the cluster
    Eigen::Vector4f centroid_eigen;
    pcl::compute3DCentroid(*cloud, cluster_indices, centroid_eigen);

    // Assign values
    centroid.x = centroid_eigen[0];
    centroid.y = centroid_eigen[1];
    centroid.z = centroid_eigen[2];

    return true;
}

// ASSOCIATING LIDAR TO 2D OBJECT DETECTION FUNCTIONS ------------------------------------------------------------------------------------------------
void ProjectionUtils::filterClusterByBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    const pcl::PointIndices& cluster_indices,
    const vision_msgs::msg::Detection2DArray& detections,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& projection_matrix,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {

    // Step 1: Compute the centroid of the cluster
    pcl::PointXYZ centroid;
    computeClusterCentroid(input_cloud, cluster_indices, centroid);

    RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Cluster centroid: x=%f, y=%f, z=%f", centroid.x, centroid.y, centroid.z);

    // Step 2: Project the centroid onto the 2D image plane
    auto projected_centroid = projectLidarToCamera3D(transform, projection_matrix, centroid);

    // If the centroid cannot be projected (e.g., it's behind the camera), return early
    if (!projected_centroid) {
        RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Centroid projection failed.");
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Projected centroid: x=%f, y=%f", projected_centroid->x, projected_centroid->y);

    // Step 3: Check if the projected centroid lies within any of the 2D bounding boxes
    for (const auto& detection : detections.detections) {
        // Get the bounding box coordinates
        double bbox_x = detection.bbox.center.position.x;
        double bbox_y = detection.bbox.center.position.y;
        double bbox_width = detection.bbox.size_x;
        double bbox_height = detection.bbox.size_y;

        RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Bounding box: center=(%f, %f), width=%f, height=%f", bbox_x, bbox_y, bbox_width, bbox_height);

        // Define the bounding box rectangle
        cv::Rect2d bbox_rect(
            bbox_x - bbox_width / 2,  // x (top-left corner)
            bbox_y - bbox_height / 2, // y (top-left corner)
            bbox_width,              // width
            bbox_height              // height
        );

        RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Bounding box rect: top-left=(%f, %f), width=%f, height=%f", bbox_rect.x, bbox_rect.y, bbox_rect.width, bbox_rect.height);

        // Check if the projected centroid lies within the bounding box
        if (bbox_rect.contains(cv::Point2d(projected_centroid->x, projected_centroid->y))) {
            // Step 4: Copy all points in the cluster to the output cloud
            RCLCPP_INFO(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Found matching bounding box");
            for (const auto& index : cluster_indices.indices) {
                output_cloud->points.push_back(input_cloud->points[index]);
            }
            RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Cluster points added to output cloud.");
            break; // Exit the loop once a matching bounding box is found
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] Projected centroid not in this bounding box.");
        }
    }
    if(output_cloud->points.empty()){
        RCLCPP_DEBUG(rclcpp::get_logger("ProjectionUtils"), "[DEBUG] No matching bounding box found for cluster.");
    }
}

// BOUNDING BOX FUNCTIONS --------------------------------------------------------------------------------------------

void ProjectionUtils::computeBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices& cluster_indices,
    pcl::PointXYZ& min_pt,
    pcl::PointXYZ& max_pt) {
        if (cloud->empty() || cluster_indices.indices.empty()) return; // Handle empty input

        // Extract the cluster points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud, cluster_indices, *cluster_cloud);

        // Perform PCA on the cluster
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cluster_cloud);
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
        Eigen::Vector4f centroid = pca.getMean();

        // Transform points into PCA-aligned space
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = eigen_vectors.transpose(); // Rotation
        transform.block<3,1>(0,3) = -eigen_vectors.transpose() * centroid.head(3); // Translation

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*cluster_cloud, transformed_cloud, transform);

        // Get min and max in PCA-aligned space
        pcl::PointXYZ min_pca, max_pca;
        pcl::getMinMax3D(transformed_cloud, min_pca, max_pca);

        // Transform back to original coordinate system
        Eigen::Matrix4f inverse_transform = transform.inverse();
        Eigen::Vector4f min_pt_transformed = inverse_transform * Eigen::Vector4f(min_pca.x, min_pca.y, min_pca.z, 1);
        Eigen::Vector4f max_pt_transformed = inverse_transform * Eigen::Vector4f(max_pca.x, max_pca.y, max_pca.z, 1);

        // Assign values
        min_pt.x = min_pt_transformed.x();
        min_pt.y = min_pt_transformed.y();
        min_pt.z = min_pt_transformed.z();

        max_pt.x = max_pt_transformed.x();
        max_pt.y = max_pt_transformed.y();
        max_pt.z = max_pt_transformed.z();

    }

