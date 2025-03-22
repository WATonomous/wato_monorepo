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
#include <pcl/features/moment_of_inertia_estimation.h>


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
    std::vector<pcl::PointIndices>& cluster_indices, 
    const vision_msgs::msg::Detection2DArray& detections,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& projection_matrix) {
    
    if (input_cloud->empty() || cluster_indices.empty()) return;  // Handle empty input

    double max_iou = 0.0;
    std::vector<pcl::PointIndices> updated_clusters;  // Store clusters with good IoU

    for (auto& cluster : cluster_indices) {
        std::vector<cv::Point2d> projected_points;

        // Project each point in the cluster to the image plane
        for (const auto& idx : cluster.indices) {
            auto projected = projectLidarToCamera(transform, projection_matrix, input_cloud->points[idx]);
            if (projected) {
                projected_points.push_back(*projected);
            }
        }

        // Compute bounding box for projected points
        if (projected_points.empty()) continue;

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
            }
        }

        // If the cluster has a good IoU score, keep it
        if (local_max_iou > max_iou) {
            max_iou = local_max_iou;
            updated_clusters.push_back(cluster);  // Add the cluster with good IoU
        }
    }
    cluster_indices = std::move(updated_clusters);  
}

// BOUNDING BOX FUNCTIONS --------------------------------------------------------------------------------------------

visualization_msgs::msg::MarkerArray ProjectionUtils::computeBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const sensor_msgs::msg::PointCloud2& msg) {

    visualization_msgs::msg::MarkerArray marker_array;

    if (cloud->empty()) return marker_array; // Handle empty input

    int id = 0;
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        Eigen::Vector4f min_point = Eigen::Vector4f::Constant(std::numeric_limits<float>::max());
        Eigen::Vector4f max_point = Eigen::Vector4f::Constant(std::numeric_limits<float>::lowest());

        // Compute min/max points manually
        for (const auto& index : cluster.indices) {
            const auto& pt = cloud->points[index];
            min_point.x() = std::min(min_point.x(), pt.x);
            min_point.y() = std::min(min_point.y(), pt.y);
            min_point.z() = std::min(min_point.z(), pt.z);

            max_point.x() = std::max(max_point.x(), pt.x);
            max_point.y() = std::max(max_point.y(), pt.y);
            max_point.z() = std::max(max_point.z(), pt.z);
        }

        // Compute bounding box center and size
        Eigen::Vector3f bbox_center = 0.5f * (min_point.head<3>() + max_point.head<3>());
        Eigen::Vector3f bbox_size = max_point.head<3>() - min_point.head<3>();

        // Calculate orientation using minAreaRect as done in getBoundingBox
        double rz = 0;

        {
            std::vector<cv::Point2f> points;
            for (const auto& index : cluster.indices) {
                cv::Point2f pt;
                pt.x = cloud->points[index].x;
                pt.y = cloud->points[index].y;
                points.push_back(pt);
            }

            cv::RotatedRect box = cv::minAreaRect(points);
            rz = box.angle * 3.14 / 180;  // Convert angle to radians

            // Update position and size with the rotated bounding box data
            bbox_center.x() = box.center.x;
            bbox_center.y() = box.center.y;
            bbox_size.x() = box.size.width;
            bbox_size.y() = box.size.height;
        }

        // Prepare Marker
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header = msg.header;
        bbox_marker.ns = "bounding_boxes";
        bbox_marker.id = id++;
        bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        bbox_marker.pose.position.x = bbox_center.x();
        bbox_marker.pose.position.y = bbox_center.y();
        bbox_marker.pose.position.z = bbox_center.z();

        // Set orientation using the calculated rotation
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, rz);  // Set only the Z-rotation
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(quat);
        bbox_marker.pose.orientation = msg_quat;

        // Size
        bbox_marker.scale.x = bbox_size.x();
        bbox_marker.scale.y = bbox_size.y();
        bbox_marker.scale.z = bbox_size.z();

        // Color
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 0.0;
        bbox_marker.color.b = 0.0;
        bbox_marker.color.a = 0.2; 

        bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_array.markers.push_back(bbox_marker);
    }

    return marker_array;
}