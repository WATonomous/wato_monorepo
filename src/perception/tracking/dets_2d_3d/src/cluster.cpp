#include "cluster.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// #include "opencv2/core/core.hpp"
// #include "opencv2/imgproc/imgproc.hpp"


typedef pcl::PointCloud<pcl::PointXYZRGB> ClusterCloud;
typedef vision_msgs::msg::BoundingBox3D BBox3D;

int Cluster::color_index = 0;
std::vector<std::vector<int>> Cluster::colors = {{0,0,255}, {0,255,0}, {255,0,0}, {255,255,0}, {0,255,255}, {255,0,255}};

Cluster::Cluster(
    const ClusterCloud::Ptr& in_cloud_ptr,
    const std::vector<int>& in_cluster_indices) : cloud_ {new ClusterCloud()}
{
    bool indexesGiven = in_cluster_indices.size() > 0;

    int max_index = (indexesGiven) ? in_cluster_indices.size() : in_cloud_ptr->points.size();

    for (int i=0; i<max_index; ++i)
    {
        int idx = (indexesGiven) ? in_cluster_indices[i] : i;

        pcl::PointXYZRGB pt;
        pt.x = in_cloud_ptr->points[idx].x;
        pt.y = in_cloud_ptr->points[idx].y;
        pt.z = in_cloud_ptr->points[idx].z;
        pt.r = in_cloud_ptr->points[idx].r;
        pt.g = in_cloud_ptr->points[idx].g;
        pt.b = in_cloud_ptr->points[idx].b;

        cloud_->emplace_back(pt);
    }

    ++color_index;
}

Cluster::Cluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud_ptr, 
    const std::vector<int>& in_cluster_indices) : cloud_ {new ClusterCloud()}
{
    bool indexesGiven = in_cluster_indices.size() > 0;

    int max_index = (indexesGiven) ? in_cluster_indices.size() : in_cloud_ptr->points.size();

    for (int i=0; i<max_index; ++i)
    {
        int idx = (indexesGiven) ? in_cluster_indices[i] : i;

        pcl::PointXYZRGB pt;
        pt.x = in_cloud_ptr->points[idx].x;
        pt.y = in_cloud_ptr->points[idx].y;
        pt.z = in_cloud_ptr->points[idx].z;
        pt.r = colors[color_index % colors.size()][0];
        pt.g = colors[color_index % colors.size()][1];
        pt.b = colors[color_index % colors.size()][2];


        cloud_->emplace_back(pt);
    }

    ++color_index;
}

// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
BBox3D Cluster::getBoundingBox()
{
    // AXIS ALIGNED BBOX
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloud_, minPoint, maxPoint);


    BBox3D bbox3d;
    bbox3d.center.position.x = (maxPoint.x + minPoint.x)/2;
    bbox3d.center.position.y = (maxPoint.y + minPoint.y)/2;
    bbox3d.center.position.z = (maxPoint.z + minPoint.z)/2;

    bbox3d.size.x = maxPoint.x - minPoint.x;
    bbox3d.size.y = maxPoint.y - minPoint.y;
    bbox3d.size.z = maxPoint.z - minPoint.z;

    return bbox3d;
}

bool Cluster::isValid(const BBox3D& b)
{
    return cloud_->size() > 0;
    // return (b.size.x > 0 && b.size.y > 0 && b.size.z > 0);
}
