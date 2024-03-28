#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cluster.hpp"
#include "projection_utils.hpp"

#include <map>

struct ClusteringParams;

class TrackingNode : public rclcpp::Node
{
public:
    TrackingNode();

private:
    std::map<std::string, ClusteringParams> clusteringParams;

    // lidar and camera frame names
    std::string lidarFrame_;
    std::string cameraFrame_;

    // synchronization utils
    std::mutex lidarCloud_mutex_;

    // camera information cached
    bool transformInited_;
    geometry_msgs::msg::TransformStamped transform_;
    sensor_msgs::msg::CameraInfo::SharedPtr camInfo_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_;

    // subscribers to camera intrinsics, lidar pts, camera feed, 2d detection boxes
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfo_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_subscriber_;

    // publish the 3d detections
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr det3d_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher2_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void readCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void receiveLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void receiveDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

    int highestIOUScoredBBox(
        const std::vector<vision_msgs::msg::BoundingBox3D> bboxes,
        const vision_msgs::msg::BoundingBox2D& detBBox,
        const std::vector<std::shared_ptr<Cluster>>& clusters);
    double overlapBoundingBox(
        const vision_msgs::msg::BoundingBox2D& bboxA, 
        const vision_msgs::msg::BoundingBox2D& bboxB);
    double iouScore(
        const vision_msgs::msg::BoundingBox2D& bboxA, 
        const vision_msgs::msg::BoundingBox2D& bboxB);

    std::map<std::string, ClusteringParams> initializeClusteringParams(
        const std::map<std::string, rclcpp::Parameter>& clustering_params_map);

    // should put somewhere else?
    template <typename T>
    T getDefaultOrValue(std::map<std::string, T> m, std::string key)
    {
        if (m.find(key) == m.end())
            return m[key];
        return m["default"];
    }

};
