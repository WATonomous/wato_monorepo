#ifndef LIDAR_DETECTOR_H
#define LIDAR_DETECTOR_H

#define BOOST_BIND_NO_PLACEHOLDERS

#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <functional>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common_msgs/msg/obstacle_list.hpp>
#include <common_msgs/msg/bounding_box.hpp>
#include <common_msgs/msg/bounding_box_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

using namespace std;

class LidarPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    int frameNum;
    int NUM_FRAMES = 10;
    std::string data_dir;
    // declare a timer to publish the point cloud
    rclcpp::TimerBase::SharedPtr timer_;

    std::string kitti_data_dir;


public:
    LidarPublisher(); 

    ~LidarPublisher(){};

    void pub_callback();
};

#endif 
