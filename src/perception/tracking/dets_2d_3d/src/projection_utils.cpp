#include "projection_utils.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
typedef std::shared_ptr<Cluster> ClusterPtr;
typedef vision_msgs::msg::BoundingBox3D BBox3D;

std::vector<double> ProjectionUtils::clustering_distances_;
std::vector<double> ProjectionUtils::clustering_thresholds_; 
double ProjectionUtils::cluster_size_min_;
double ProjectionUtils::cluster_size_max_;
double ProjectionUtils::cluster_merge_threshold_;

void ProjectionUtils::pointsInBbox(
    const Cloud::Ptr& inlierCloud,
    const Cloud::Ptr& lidarCloud, 
    const std::vector<geometry_msgs::msg::Point>& projs2d,
    const vision_msgs::msg::BoundingBox2D& bbox)
{
    for (size_t i=0; i<projs2d.size(); ++i)
    {
        // P * [x y z 1]^T, P is row major
        if (isPointInBbox(projs2d[i], bbox))
            inlierCloud->push_back(lidarCloud->points[i]);
    }
}
  
bool ProjectionUtils::isPointInBbox(const geometry_msgs::msg::Point& pt, const vision_msgs::msg::BoundingBox2D& bbox)
{
    double padding = 0;

    if (bbox.center.position.x - bbox.size_x/2 - padding < pt.x && pt.x < bbox.center.position.x + bbox.size_x/2 + padding
       && bbox.center.position.y - bbox.size_y/2 - padding < pt.y && pt.y < bbox.center.position.y + bbox.size_y/2 + padding)
    {
        return true;
    }
    return false;

}

std::optional<geometry_msgs::msg::Point> ProjectionUtils::projectLidarToCamera(
    const geometry_msgs::msg::TransformStamped& transform,
    const std::array<double, 12>& p, 
    const pcl::PointXYZ& pt)
{
    // lidar to camera frame
    auto trans_pt = geometry_msgs::msg::Point();
    auto orig_pt = geometry_msgs::msg::Point();
    orig_pt.x = pt.x;
    orig_pt.y = pt.y;
    orig_pt.z = pt.z;

    tf2::doTransform(orig_pt, trans_pt, transform);

    if (trans_pt.z < 1) return std::nullopt;

    // camera frame to camera 2D projection
    double u = p[0] * trans_pt.x + p[1] * trans_pt.y + p[2] * trans_pt.z + p[3];
    double v = p[4] * trans_pt.x + p[5] * trans_pt.y + p[6] * trans_pt.z + p[7];
    double w = p[8] * trans_pt.x + p[9] * trans_pt.y + p[10] * trans_pt.z + p[11];

    auto proj_pt = geometry_msgs::msg::Point();
    proj_pt.x = u/w;
    proj_pt.y = v/w;

    // check if inside camera frame bounds
    if (proj_pt.x > 0 && proj_pt.x < 1600 && proj_pt.y > 0 && proj_pt.y < 900) return proj_pt;
    return std::nullopt;
}

// https://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.html
void ProjectionUtils::removeFloor(const Cloud::Ptr& lidarCloud, const Cloud::Ptr& cloud_filtered)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Create the filtering object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(0.1);
    seg.setDistanceThreshold(0.5); // floor distance
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(lidarCloud);
    seg.segment(*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(lidarCloud);
    extract.setIndices(inliers);

    // Extract non-ground returns
    extract.setNegative (true);
    extract.filter(*cloud_filtered);
}

std::pair<std::vector<ClusterPtr>, std::vector<BBox3D>> ProjectionUtils::getClusteredBBoxes(const Cloud::Ptr& lidarCloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (size_t i=0; i<cloud_segments_array.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_segments_array[i] = temp;
    }

    // segment points into spherical shells from point cloud origin
    for (const pcl::PointXYZ& pt : lidarCloud->points) 
    {        
        float origin_distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if (origin_distance < clustering_distances_[0])
            cloud_segments_array[0]->points.push_back(pt);
        else if (origin_distance < clustering_distances_[1])
            cloud_segments_array[1]->points.push_back(pt);
        else if (origin_distance < clustering_distances_[2])
            cloud_segments_array[2]->points.push_back(pt);
        else if (origin_distance < clustering_distances_[3])
            cloud_segments_array[3]->points.push_back(pt);
        else
            cloud_segments_array[4]->points.push_back(pt);
    }

    // get largest cluster in each shell
    std::vector<ClusterPtr> all_clusters;
    for (unsigned int i = 1; i < cloud_segments_array.size(); i++) 
    {
        // add clusters from each shell 
        std::vector<ClusterPtr> local_clusters = clusterAndColor(cloud_segments_array[i], clustering_thresholds_[i]);
        all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }

    // merge clusters if possible, do this twice?
    std::vector<ClusterPtr> mid_clusters = (all_clusters.size() > 0) 
        ?   ProjectionUtils::checkAllForMerge(all_clusters, cluster_merge_threshold_)
        :   all_clusters;
    std::vector<ClusterPtr> final_clusters = (mid_clusters.size() > 0)
        ?   ProjectionUtils::checkAllForMerge(mid_clusters, cluster_merge_threshold_)
        :   mid_clusters;

    // get boundingboxes for each & return all possible 3d bboxes (if valid)
    std::vector<BBox3D> bboxes;
    for (const ClusterPtr& cluster : final_clusters)
    {
        BBox3D b = cluster->getBoundingBox();
        if (cluster->isValid())
            bboxes.emplace_back(b);
    }
    return std::pair<std::vector<ClusterPtr>, std::vector<BBox3D>>{final_clusters, bboxes};
}

std::vector<ClusterPtr> ProjectionUtils::clusterAndColor(
    const Cloud::Ptr& in_cloud_ptr, double in_max_cluster_distance) 
{
    std::vector<ClusterPtr> clusters;
    if(in_cloud_ptr->size() == 0) return clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // create 2d pc, by copying & making it flat
    Cloud::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> cluster_indices;

    // perform clustering on 2d cloud : https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(in_max_cluster_distance);
    ec.setMinClusterSize(cluster_size_min_);
    ec.setMaxClusterSize(cluster_size_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);


    // add pts at clustered indexes to cluster
    for (const auto& cluster : cluster_indices)
    {
        ClusterPtr cloud_cluster = std::make_shared<Cluster>(in_cloud_ptr, cluster.indices);
        clusters.emplace_back(cloud_cluster);
    }

    return clusters;
}


void ProjectionUtils::checkClusterMerge(
    size_t in_cluster_id, const std::vector<ClusterPtr> &in_clusters,
    std::vector<bool>& in_out_visited_clusters,
    std::vector<size_t>& out_merge_indices, double in_merge_threshold) 
{

    pcl::PointXYZ point_a = in_clusters[in_cluster_id]->getCentroid();

    for (size_t i = 0; i < in_clusters.size(); i++) {
        if (i != in_cluster_id && !in_out_visited_clusters[i]) 
        {
            pcl::PointXYZ point_b = in_clusters[i]->getCentroid();
            double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));

            if (distance <= in_merge_threshold) 
            {
                in_out_visited_clusters[i] = true;
                out_merge_indices.push_back(i);
                // look for all other clusters that can be merged with this merge-able cluster
                checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
            }
        }
    }
}


ClusterPtr ProjectionUtils::mergeClusters(
    const std::vector<ClusterPtr>& in_clusters,
    const std::vector<size_t>& in_merge_indices,
    std::vector<bool> &in_out_merged_clusters) 
{
    ColorCloud merged_cloud;    

    // for each cluster in merge cloud indices, merge into larger cloud
    for (size_t i = 0; i < in_merge_indices.size(); i++) 
    {
        merged_cloud += *(in_clusters[in_merge_indices[i]]->getCloud());
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }

    Cloud merged_cloud_uncoloured;
    pcl::copyPointCloud(merged_cloud, merged_cloud_uncoloured);

    Cloud::Ptr merged_cloud_ptr(new Cloud(merged_cloud_uncoloured));

    std::vector<int> temp;
    ClusterPtr merged_cluster = std::make_shared<Cluster>(merged_cloud_ptr, temp);
    return merged_cluster;
}

std::vector<ClusterPtr> ProjectionUtils::checkAllForMerge(
    const std::vector<ClusterPtr>& in_clusters, float in_merge_threshold) 
{
    std::vector<ClusterPtr> out_clusters;

    std::vector<bool> visited_clusters(in_clusters.size(), false);
    std::vector<bool> merged_clusters(in_clusters.size(), false);

    for (size_t i = 0; i < in_clusters.size(); i++) {
        if (!visited_clusters[i]) {
            visited_clusters[i] = true;

            std::vector<size_t> merge_indices;
            checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
            ClusterPtr mergedCluster = mergeClusters(in_clusters, merge_indices, merged_clusters);

            out_clusters.emplace_back(mergedCluster);
        }
    }
    for (size_t i = 0; i < in_clusters.size(); i++) {
        // check for clusters not merged, add them to the output
        if (!merged_clusters[i]) {
            out_clusters.push_back(in_clusters[i]);
        }
    }

    return out_clusters;
}
