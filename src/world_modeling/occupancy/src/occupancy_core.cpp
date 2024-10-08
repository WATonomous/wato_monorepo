#include <string>
#include <cstring>
#include <vector>

#include "occupancy_core.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Constructor
OccupancyCore::OccupancyCore() {}

sensor_msgs::msg::PointCloud2 OccupancyCore::remove_z_dimension(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 output_cloud;

    // Initialize the output point cloud
    output_cloud.header = msg->header;
    output_cloud.height = 1; // Height of point cloud is 1 for 2D points
    output_cloud.width = msg->width;
    // output_cloud.is_dense = true; // TODO Sophie: true or false?

    // Redefine fields for a 2D point cloud
    output_cloud.fields.resize(2);
    output_cloud.fields[0] = sensor_msgs::msg::PointField{"x", 0, sensor_msgs::msg::PointField::FLOAT32, 1};
    output_cloud.fields[1] = sensor_msgs::msg::PointField{"y", 4, sensor_msgs::msg::PointField::FLOAT32, 1};

    output_cloud.point_step = 8; // Each point takes 8 bytes, 4 bytes each for x and y
    output_cloud.row_step = output_cloud.point_step * output_cloud.width;

    // Resize data for 2D points
    output_cloud.data.resize(output_cloud.row_step * output_cloud.height);
    
    // Copy x and y values from the input cloud using the iterator
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (int i = 0; i < msg->width; ++i) {
        std::memcpy(&output_cloud.data[i * output_cloud.point_step], &(*iter_x), sizeof(float)); // Copy x
        std::memcpy(&output_cloud.data[i * output_cloud.point_step + 4], &(*iter_y), sizeof(float)); // Copy y
        ++iter_x;
        ++iter_y;
    }
    return output_cloud;
}
