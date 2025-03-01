#ifndef LIDAR_OVERLAY_HPP
#define LIDAR_OVERLAY_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "projection_utils.hpp" 

class LidarImageOverlay : public rclcpp::Node {
    public:
        LidarImageOverlay();

    private:
        // IMAGE -----------------------------------------------------------------------------------------------------------
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        cv_bridge::CvImagePtr image_data_;
        std::array<double, 12> projection_matrix_;

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        // LIDAR -----------------------------------------------------------------------------------------------------------
        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        sensor_msgs::msg::PointCloud2 latest_lidar_msg_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_;

        // DETECTIONS ------------------------------------------------------------------------------------------------------
        void detsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

        // PROJECTION ------------------------------------------------------------------------------------------------------
        std::optional<cv::Point2d> projectLidarToCamera(
            const geometry_msgs::msg::TransformStamped& transform,
            const std::array<double, 12>& p,
            const pcl::PointXYZ& pt,
            int image_width = 1600,
            int image_height = 900);

        // SUBSCRIBERS -----------------------------------------------------------------------------------------------------
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // PUBLISHERS ------------------------------------------------------------------------------------------------------
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;
};

#endif 