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
#include <tf2_eigen/tf2_eigen.hpp>


#include <random>
#include <future>
#include <thread>
#include <mutex>

void ProjectionUtils::removeGroundPlane(PointCloud::Ptr& cloud, float distanceThreshold, int maxIterations) {
    if (cloud->empty()) return;

    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    ransac.setDistanceThreshold(distanceThreshold);
    ransac.setMaxIterations(maxIterations);
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
    geometry_msgs::msg::PointStamped lidar_point;
    lidar_point.point.x = pt.x;
    lidar_point.point.y = pt.y;
    lidar_point.point.z = pt.z;

    // Transform LiDAR point to camera frame using tf2::doTransform
    geometry_msgs::msg::PointStamped camera_point;
    tf2::doTransform(lidar_point, camera_point, transform);

    // Reject points behind the camera (z < 1)
    if (camera_point.point.z < 1) {
        return std::nullopt;
    }

    // Convert the projection matrix (std::array) to Eigen::Matrix<double, 3, 4>
    Eigen::Matrix<double, 3, 4> projection_matrix;
    projection_matrix << p[0], p[1], p[2], p[3],
                         p[4], p[5], p[6], p[7],
                         p[8], p[9], p[10], p[11];

    // Convert camera point to Eigen vector (homogeneous coordinates)
    Eigen::Vector4d camera_point_eigen(camera_point.point.x, camera_point.point.y, camera_point.point.z, 1.0);

    // Project the point onto the image plane using the projection matrix
    Eigen::Vector3d projected_point = projection_matrix * camera_point_eigen;

    // Normalize the projected coordinates
    cv::Point2d proj_pt;
    proj_pt.x = projected_point.x() / projected_point.z();
    proj_pt.y = projected_point.y() / projected_point.z();

    // Check if the projected point is within the image bounds
    if (proj_pt.x >= 0 && proj_pt.x < image_width_ && proj_pt.y >= 0 && proj_pt.y < image_height_) {
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


void ProjectionUtils::filterClusterbyDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                            const std::vector<pcl::PointIndices>& cluster_indices,
                                            double densityWeight,
                                            double sizeWeight,
                                            double distanceWeight,
                                            double scoreThreshold) {

    if (cloud->empty() || cluster_indices.empty()) return; 

    // Define maximum expected values for normalization
    const double max_density = 700.0;   // Maximum density (points/m³)
    const double max_size = 10.0;        // Maximum cluster size (m³)
    const double max_distance = 60.0;   // Maximum distance (meters)

    std::vector<pcl::PointIndices> filtered_clusters;

    for (const auto& clusters : cluster_indices) {
        if (clusters.indices.size() < 10) continue; // Skip small clusters

        // Initialize min and max values for cluster bounds
        double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
        double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();
        double total_distance = 0.0;

        // Calculate cluster bounds and total distance from origin
        for (const auto& index : clusters.indices) {
            const auto& pt = cloud->points[index];
            min_x = std::min(min_x, static_cast<double>(pt.x));
            max_x = std::max(max_x, static_cast<double>(pt.x));
            min_y = std::min(min_y, static_cast<double>(pt.y));
            max_y = std::max(max_y, static_cast<double>(pt.y));
            min_z = std::min(min_z, static_cast<double>(pt.z));
            max_z = std::max(max_z, static_cast<double>(pt.z));
            total_distance += std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        }

        // Calculate cluster size (bounding box diagonal)
        double cluster_size = std::sqrt((max_x - min_x) * (max_x - min_x) +
                              (max_y - min_y) * (max_y - min_y) +
                              (max_z - min_z) * (max_z - min_z));

        // Calculate cluster density (points per unit volume)
        double cluster_volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
        double density = cluster_volume > 0 ? clusters.indices.size() / cluster_volume : 0;

        // Calculate average distance from origin
        double avg_distance = total_distance / clusters.indices.size();

        // Normalize the factors
        double normalized_density = density / max_density;
        double normalized_size = cluster_size / max_size;
        double normalized_distance = avg_distance / max_distance;

        // Compute the weighted score
        double score = (normalized_density * densityWeight) +
                       (normalized_size * sizeWeight) +
                       (normalized_distance * distanceWeight);

        // If the score is below the threshold, keep the cluster
        if (score < scoreThreshold) {
            filtered_clusters.push_back(clusters);
        }
    }

    // Replace the original cluster indices with the filtered ones
    const_cast<std::vector<pcl::PointIndices>&>(cluster_indices) = filtered_clusters;
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

bool ProjectionUtils::pointIn2DBoundingBox(
    const pcl::PointXYZ& point, 
    const vision_msgs::msg::Detection2DArray& detections,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& projection_matrix) {

    auto proj_pt = projectLidarToCamera(transform, projection_matrix, point);
    if (!proj_pt.has_value()) {
        return false;
    }

    // Check if the projected point lies within any of the bounding boxes
    for (const auto& detection : detections.detections) {
        const auto& bbox = detection.bbox;

        // Bounding box coordinates
        double x_min = bbox.center.position.x - bbox.size_x / 2;
        double x_max = bbox.center.position.x + bbox.size_x / 2;
        double y_min = bbox.center.position.y - bbox.size_y / 2;
        double y_max = bbox.center.position.y + bbox.size_y / 2;

        // Check if the projected point lies within the bounding box
        if (proj_pt->x >= x_min && proj_pt->x <= x_max &&
            proj_pt->y >= y_min && proj_pt->y <= y_max) {
            return true;
        }
    }
    return false;
}

void ProjectionUtils::filterClusterByBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const vision_msgs::msg::Detection2DArray& detections,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& projection_matrix,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
    
    if (input_cloud->empty() || cluster_indices.empty()) return;  // Handle empty input
    output_cloud->clear(); 

    std::mutex mtx;  
    double max_iou = 0.0;
    pcl::PointIndices best_cluster;

    std::vector<std::future<void>> tasks;  // Store futures for async tasks

    for (const auto& cluster : cluster_indices) {
        tasks.push_back(std::async(std::launch::async, [&]() {
            std::vector<cv::Point2d> projected_points;

            // Project each point in the cluster to the image plane
            for (const auto& idx : cluster.indices) {
                auto projected = projectLidarToCamera(transform, projection_matrix, input_cloud->points[idx]);
                if (projected) {
                    projected_points.push_back(*projected);
                }
            }

            // Compute bounding box for projected points
            if (projected_points.empty()) return;

            double min_x = std::numeric_limits<double>::max(), max_x = 0;
            double min_y = std::numeric_limits<double>::max(), max_y = 0;

            for (const auto& pt : projected_points) {
                min_x = std::min(min_x, pt.x);
                max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y);
                max_y = std::max(max_y, pt.y);
            }

            cv::Rect cluster_bbox(min_x, min_y, max_x - min_x, max_y - min_y);
            double local_max_iou = 0.0;
            pcl::PointIndices local_best_cluster;

            // Find the detection with the highest IoU
            for (const auto& detection : detections.detections) {
                const auto& bbox = detection.bbox;
                cv::Rect detection_bbox(bbox.center.position.x - bbox.size_x / 2,
                    bbox.center.position.y - bbox.size_y / 2,
                    bbox.size_x, bbox.size_y);

                double intersection_area = (cluster_bbox & detection_bbox).area();
                double union_area = cluster_bbox.area() + detection_bbox.area() - intersection_area;
                double iou = (union_area > 0) ? (intersection_area / union_area) : 0.0;

                if (iou > local_max_iou) {
                    local_max_iou = iou;
                    local_best_cluster = cluster;
                }
            }

            // Safely update global best cluster
            std::lock_guard<std::mutex> lock(mtx);
            if (local_max_iou > max_iou) {
                max_iou = local_max_iou;
                best_cluster = local_best_cluster;
            }
        }));
    }

    // Wait for all threads to finish
    for (auto& task : tasks) {
        task.get();
    }

    // Copy the best cluster to the output cloud
    if (!best_cluster.indices.empty()) {
        for (const auto& idx : best_cluster.indices) {
            output_cloud->push_back(input_cloud->points[idx]);
        }
    }
}

void ProjectionUtils::projectAndDrawPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const vision_msgs::msg::Detection2DArray& detections,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& projection_matrix,
    cv::Mat& image) {

    if (cloud->empty()) return;

    for (const auto& point : cloud->points) {
        auto projected_point = projectLidarToCamera(transform, projection_matrix, point);
        if (projected_point) {
            cv::circle(image, *projected_point, 2, cv::Scalar(0, 255, 0), -1);
        }
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

