#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include <vector>

class Cluster
{
public:
    // make cloud from only pts that have in "in_cluster_indices"
    Cluster(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud_ptr, 
        const std::vector<int>& in_cluster_indices);

    Cluster(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud_ptr, 
        const std::vector<int>& in_cluster_indices);

    pcl::PointXYZ getCentroid() { return centroid_; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() { return cloud_; }
    vision_msgs::msg::BoundingBox3D getBoundingBox();

    bool isValid(const vision_msgs::msg::BoundingBox3D& b);


private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointXYZ centroid_;

    // stores the min_x, min_y, min_z
    pcl::PointXYZ max_point_;
    pcl::PointXYZ min_point_;

    static int color_index;
    static std::vector<std::vector<int>> colors;
};
