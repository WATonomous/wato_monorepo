#pragma once

#include "cluster.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <optional>

struct ClusteringParams 
{
    // segment by distance
    std::vector<double> clustering_distances;
    // Map of the nearest neighbor distance threshold for each segment, for each detection
    std::vector<double> clustering_thresholds;

    double cluster_size_min;
    double cluster_size_max;
    double cluster_merge_threshold;
};

// main helper functions used by the node -- should all be static
class ProjectionUtils
{
public:
        static void pointsInBbox(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& inlierCloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidarCloud, 
                const std::vector<geometry_msgs::msg::Point>& projs2d,
                const vision_msgs::msg::BoundingBox2D& bbox);

        // P : projection matrix, pt : 3D lidar pt, transform: transform from lidar to camera frame
        static std::optional<geometry_msgs::msg::Point> projectLidarToCamera(
                const geometry_msgs::msg::TransformStamped& transform,
                const std::array<double, 12>& P, 
                const pcl::PointXYZ& pt);

        static bool isPointInBbox(
                const geometry_msgs::msg::Point& pt, 
                const vision_msgs::msg::BoundingBox2D& bbox);

        static void removeFloor(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidarCloud, 
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered);

        static std::pair<std::vector<std::shared_ptr<Cluster>>, std::vector<vision_msgs::msg::BoundingBox3D>> getClusteredBBoxes(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidarCloud, const ClusteringParams& clusteringParams);

private:

        static std::vector<std::shared_ptr<Cluster>> clusterAndColor(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud_ptr, 
                double in_max_cluster_distance, double cluster_size_min, double cluster_size_max);

        static void checkClusterMerge(
                size_t in_cluster_id, const std::vector<std::shared_ptr<Cluster>> &in_clusters,
                std::vector<bool> &in_out_visited_clusters,
                std::vector<size_t> &out_merge_indices, double in_merge_threshold);

        static std::shared_ptr<Cluster> mergeClusters(
                const std::vector<std::shared_ptr<Cluster>>& in_clusters,
                const std::vector<size_t>& in_merge_indices,
                std::vector<bool>& in_out_merged_clusters);

        static std::vector<std::shared_ptr<Cluster>> checkAllForMerge(
                const std::vector<std::shared_ptr<Cluster>>& in_clusters, 
                float in_merge_threshold);
};
