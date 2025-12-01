#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization::slam {

struct PreprocessParams {
    double voxel_leaf_size = 0.2;
    double min_range = 1.0;
    double max_range = 100.0;
};

class CloudPreprocessor {
    public:
        explicit CloudPreprocessor(const PreprocessorParams &params)
        pcl::PointCloud<pcl::PointXYZI>::Ptr process(const sensor_msgs::msgs::PointCloud2 &msg) const;
    private:
        PreprocessorParams params_;
};

}