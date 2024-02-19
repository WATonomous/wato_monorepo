#include <chrono>
#include <memory>
#include <vector>

#include "occupancy_segmentation_node.hpp"

OccupancySegmentationNode::OccupancySegmentationNode()
: Node("occupancy_segmentation")
{
  //http://wiki.ros.org/message_filters I am tempted to use this method to combine camera callbacks, may implement in the future

  // /LIDAR_TOP (sensor_msgs/msg/PointCloud2)
  // /CAM_BACK/camera_info (sensor_msgs/msg/CameraInfo)
  // /CAM_BACK_LEFT/camera_info
  // /CAM_BACK_RIGHT/camera_info
  // /CAM_FRONT/camera_info
  // /CAM_FRONT_LEFT/camera_info
  // /CAM_FRONT_RIGHT/camera_info

  pt_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/LIDAR_TOP", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::lidar_callback, this,
      std::placeholders::_1));

  cam_back_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_BACK/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_back_callback, this,
      std::placeholders::_1));
  cam_back_left_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_BACK_LEFT/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_back_left_callback, this,
      std::placeholders::_1));
  cam_back_right_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_BACK_RIGHT/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_back_right_callback, this,
      std::placeholders::_1));
  cam_front_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_FRONT/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_front_callback, this,
      std::placeholders::_1));
  cam_front_left_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_FRONT_LEFT/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_front_left_callback, this,
      std::placeholders::_1));
  cam_front_right_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/CAM_FRONT_RIGHT/camera_info", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::cam_front_right_callback, this,
      std::placeholders::_1));
}
void OccupancySegmentationNode::lidar_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Raw Frequency(msg/s)");
}

void OccupancySegmentationNode::cam_back_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}
void OccupancySegmentationNode::cam_back_left_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}
void OccupancySegmentationNode::cam_back_right_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}
void OccupancySegmentationNode::cam_front_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}
void OccupancySegmentationNode::cam_front_left_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}
void OccupancySegmentationNode::cam_front_right_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing Nothing");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
