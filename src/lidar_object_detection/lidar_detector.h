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

#include "cuda_runtime.h"
#include "params.h"
#include "cloud_data.h"
#include "pointpillar.h"

using namespace std;

class LidarDetector : public rclcpp::Node {
private:
    rclcpp::Publisher<common_msgs::msg::ObstacleList>::SharedPtr lidar_detection_pub;
    rclcpp::Publisher<common_msgs::msg::BoundingBoxArray>::SharedPtr debug_boxes_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;

    cudaStream_t stream;
    std::mutex lidar_cloud_mutex_;
    std::mutex boxes_mutex_;
    std::mutex pp_mutex_;
    std_msgs::msg::Header pc_header;
    std::string output_frame_name;
    std::string dump_pcd_path = "/home/docker";
    std::deque<CloudData> cloud_data_buff_;
    CloudData current_cloud_data_;
    std::vector<Bndbox> nms_pred;
    std::shared_ptr<PointPillar> pointpillar;

public:
    LidarDetector(); 

    ~LidarDetector(){};

    // Callback. Note that it is assumed race conditions do not exist, because
    // the callbacks are running by a single thread and is processed by
    // single-thread ROS spinning:
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    void lidarPointsCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr in_sensor_cloud
    );

    void publishVis(const std::vector<Bndbox> &boxes);
};

#endif 
