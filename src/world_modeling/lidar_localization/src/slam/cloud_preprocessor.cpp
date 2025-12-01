#include "lidar_localization/slam/cloud_preprocessor.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_localization::slam {

CloudPreprocessor::CloudPreprocessor(const PreprocessorParams &params)
    :params_(params) {}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPreprocessor::process(const sensor_msgs::msgs::PointCloud2 &msg) const
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(); //shared pointer for cloud
    //convert ROS message tand fills point cloud
    pcl::from_ROSMsg(msg, *cloud);

    //range filter
    pcl::PointCloud<pcl::PointXYZI>::ptr filtered(new pcl::PointCloud<pcl::PointXYZI>>);
    filtered->reserve(cloud->size()); // allocates memory for size of pointcloud

    for (const auto &pt: cloud->points)
    {
        double r2 = pt.x*pt.x + pt.y * pt.y + pt.z * pt.z; //distance^2 of point
        if(r2<params_.min_range * params_.min_range||
        r2 > params_.max_range * params_.max_range)
        {
            continue;
        }
        filtered->push_back(pt);
    }

    //voxel downsampling   
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(params_.voxel_leaf_size,
        params_.voxel_leaf_size,
        params_.voxel_leaf_size,
    );

    voxel.setInputCloud(filtered); //populate voxelgrid with filtered pointcloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter(*downsampled);

    return downsampled;


}


}