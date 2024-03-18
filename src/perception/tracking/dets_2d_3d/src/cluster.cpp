#include "cluster.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZRGB> ClusterCloud;
typedef vision_msgs::msg::BoundingBox3D BBox3D;

int Cluster::color_index = 0;
std::vector<std::vector<int>> Cluster::colors = {{0,0,255}, {0,255,0}, {255,0,0}, {255,255,0}, {0,255,255}, {255,0,255}};

Cluster::Cluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud_ptr, 
    const std::vector<int>& in_cluster_indices) : cloud_ {new ClusterCloud()}
{
    bool indexesGiven = in_cluster_indices.size() > 0;

    size_t max_index = (indexesGiven) ? in_cluster_indices.size() : in_cloud_ptr->points.size();

    min_point_.x = std::numeric_limits<double>::max();
    min_point_.y = std::numeric_limits<double>::max();
    min_point_.z = std::numeric_limits<double>::max();

    max_point_.x = -std::numeric_limits<double>::max();
    max_point_.y = -std::numeric_limits<double>::max();
    max_point_.z = -std::numeric_limits<double>::max();

    for (size_t i=0; i<max_index; ++i)
    {
        size_t idx = (indexesGiven) ? in_cluster_indices[i] : i;

        pcl::PointXYZRGB pt;
        pt.x = in_cloud_ptr->points[idx].x;
        pt.y = in_cloud_ptr->points[idx].y;
        pt.z = in_cloud_ptr->points[idx].z;
        pt.r = colors[color_index % colors.size()][0];
        pt.g = colors[color_index % colors.size()][1];
        pt.b = colors[color_index % colors.size()][2];

        if (pt.x < min_point_.x) min_point_.x = pt.x;
        if (pt.y < min_point_.y) min_point_.y = pt.y;
        if (pt.z < min_point_.z) min_point_.z = pt.z;

        if (pt.x > max_point_.x) max_point_.x = pt.x;
        if (pt.y > max_point_.y) max_point_.y = pt.y;
        if (pt.z > max_point_.z) max_point_.z = pt.z;


        cloud_->emplace_back(pt);
    }

    ++color_index;
}

BBox3D Cluster::getBoundingBox()
{
    BBox3D bounding_box_;
    
    if (cloud_->size() == 0) return bounding_box_;

    double length_ = max_point_.x - min_point_.x;
    double width_ = max_point_.y - min_point_.y;
    double height_ = max_point_.z - min_point_.z;

    bounding_box_.center.position.x = min_point_.x + length_ / 2;
    bounding_box_.center.position.y = min_point_.y + width_ / 2;
    bounding_box_.center.position.z = min_point_.z + height_ / 2;

    bounding_box_.size.x = ((length_ < 0) ? -1 * length_ : length_);
    bounding_box_.size.y = ((width_ < 0) ? -1 * width_ : width_);
    bounding_box_.size.z = ((height_ < 0) ? -1 * height_ : height_);

    // pose estimation
    double rz = 0;

    {
        std::vector<cv::Point2f> points;
        for (unsigned int i = 0; i < cloud_->points.size(); i++) 
        {
            cv::Point2f pt;
            pt.x = cloud_->points[i].x;
            pt.y = cloud_->points[i].y;
            points.push_back(pt);
        }

        cv::RotatedRect box = cv::minAreaRect(points);
        rz = box.angle * 3.14 / 180;
        bounding_box_.center.position.x = box.center.x;
        bounding_box_.center.position.y = box.center.y;
        bounding_box_.size.x = box.size.width;
        bounding_box_.size.y = box.size.height;
    }

    // set bounding box direction
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, rz);

    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(quat);
    bounding_box_.center.orientation = msg_quat;

    std::cout << cv::getBuildInformation() << std::endl;

    return bounding_box_;
}

bool Cluster::isValid()
{
    return cloud_->size() > 0;
}
