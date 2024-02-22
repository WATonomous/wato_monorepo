#include "det_utils.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

// segment by distance (5 sections)
// 0 => 0-15m d=0.5, 1 => 15-30 d=1, 2 => 30-45 d=1.6, 3 => 45-60 d=2.1, 4 => >60   d=2.6
std::vector<double> DetUtils::_clustering_distances = {5, 30, 45, 60};

//Nearest neighbor distance threshold for each segment
std::vector<double> DetUtils::_clustering_thresholds = {0.5, 1.1, 1.6, 2.1, 2.6}; 

double DetUtils::cluster_size_min_ = 20;
double DetUtils::cluster_size_max_ = 100000;
double DetUtils::cluster_merge_threshold_ = 1.5;

void DetUtils::pointsInBbox(
    const Cloud::Ptr& inlierCloud,
    const Cloud::Ptr& lidarCloud, 
    const std::vector<geometry_msgs::msg::Point>& projs2d,
    const vision_msgs::msg::BoundingBox2D& bbox)
{
    for (int i=0; i<projs2d.size(); ++i)
    {
        // P * [x y z 1]^T, P is row major; eigen is probably overkill
        // check if projected point is in the bbox
        if (isPointInBbox(projs2d[i], bbox))
        {
            inlierCloud->push_back(lidarCloud->points[i]);
        }

    }
}

bool DetUtils::isPointInBbox(const geometry_msgs::msg::Point& pt, const vision_msgs::msg::BoundingBox2D& bbox)
{
    double padding = 10;

    // if (bbox.center.position.x - bbox.size_x/2 - padding < pt.x && pt.x < bbox.center.position.x + bbox.size_x/2 + padding
    //    && bbox.center.position.y - bbox.size_y/2 - padding < pt.y && pt.y < bbox.center.position.y + bbox.size_y/2 + padding)
    // {
    //     return true;
    // }

    if (pt.x > 0 && pt.x < 1600 && pt.y > 0 && pt.y < 900)
        return true;
    return false;
}

/*

1266.417203046554       0.0                 816.2670197447984   0.0
0.0                     1266.417203046554   491.50706579294757  0.0                 
0.0                     0.0                 1.0                 0.0


*/

geometry_msgs::msg::Point DetUtils::projectLidarToCamera(
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

    // camera frame to camera 2D projection
    double u = p[0] * trans_pt.x + p[1] * trans_pt.y + p[2] * trans_pt.z + p[3];
    double v = p[4] * trans_pt.x + p[5] * trans_pt.y + p[6] * trans_pt.z + p[7];
    double w = p[8] * trans_pt.x + p[9] * trans_pt.y + p[10] * trans_pt.z + p[11];

    auto proj_pt = geometry_msgs::msg::Point();
    proj_pt.x = u/w;
    proj_pt.y = v/w;
    proj_pt.z = w; // needed to check if front/behind camera

    return proj_pt;
}

// https://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.html
void DetUtils::removeFloor(const Cloud::Ptr& lidarCloud, const Cloud::Ptr& cloud_filtered)
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

// https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
std::pair<std::vector<ClusterPtr>, std::vector<BBox3D>> DetUtils::getClusteredBBoxes(const Cloud::Ptr& lidarCloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (int i=0; i<cloud_segments_array.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_segments_array[i] = temp;
    }

    // segment points into spherical shells from point cloud origin
    for (const pcl::PointXYZ& pt : lidarCloud->points) 
    {        
        float origin_distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if (origin_distance < _clustering_distances[0])
            cloud_segments_array[0]->points.push_back(pt);
        else if (origin_distance < _clustering_distances[1])
            cloud_segments_array[1]->points.push_back(pt);
        else if (origin_distance < _clustering_distances[2])
            cloud_segments_array[2]->points.push_back(pt);
        else if (origin_distance < _clustering_distances[3])
            cloud_segments_array[3]->points.push_back(pt);
        else
            cloud_segments_array[4]->points.push_back(pt);
    }

    // get largest cluster in each shell
    std::vector<ClusterPtr> all_clusters;
    for (unsigned int i = 1; i < cloud_segments_array.size(); i++) 
    {
        // add clusters from each shell 
        std::vector<ClusterPtr> local_clusters = clusterAndColor(cloud_segments_array[i], _clustering_thresholds[i]);
        all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }

    // merge clusters if possible, do this twice?
    std::vector<ClusterPtr> mid_clusters = (all_clusters.size() > 0) 
        ?   DetUtils::checkAllForMerge(all_clusters, cluster_merge_threshold_)
        :   all_clusters;
    std::vector<ClusterPtr> final_clusters = (mid_clusters.size() > 0)
        ?   DetUtils::checkAllForMerge(mid_clusters, cluster_merge_threshold_)
        :   mid_clusters;

    // std::vector<ClusterPtr> final_clusters = all_clusters;

    // get boundingboxes for each & return all possible 3d bboxes (if valid)
    std::vector<BBox3D> bboxes;
    for (const ClusterPtr& cluster : final_clusters)
    {
        BBox3D b = cluster->getBoundingBox();
        if (cluster->isValid(b))
            bboxes.emplace_back(b);
    }


    // TEMP - RETURN RGB CLOUD WITH ALL CLUSTERS (not just the same merged c)
    // Cloud merged_cloud; 
    // for (const ClusterPtr& clusterPtr : final_clusters)
    // {
    //     merged_cloud += *(clusterPtr->getCloud());
    // }

    return std::pair<std::vector<ClusterPtr>, std::vector<BBox3D>>{final_clusters, bboxes};
}

std::vector<ClusterPtr> DetUtils::clusterAndColor(
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

    // perform clustering on 2d cloud
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


void DetUtils::checkClusterMerge(
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


ClusterPtr DetUtils::mergeClusters(
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

std::vector<ClusterPtr> DetUtils::checkAllForMerge(
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
