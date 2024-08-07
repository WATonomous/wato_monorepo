#include <memory>
#include <iostream>
#include "occupancy_segmentation_node.hpp"

OccupancySegmentationNode::OccupancySegmentationNode() : Node("occupancy_segmentation") {
  _subscriber =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&OccupancySegmentationNode::subscription_callback, this, std::placeholders::_1));

  _ground_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/ground_points", 10);
  _nonground_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/nonground_points", 10);

}

void OccupancySegmentationNode::subscription_callback(const  sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud){
  pcl::PointCloud<PointXYZIRT> temp_cloud;
  RCLCPP_INFO(this -> get_logger(), "Header incoming: %s", lidar_cloud -> header.frame_id.c_str());
  pcl::fromROSMsg(*lidar_cloud, temp_cloud);

  pcl::PointCloud<PointXYZIRT> ground;
  pcl::PointCloud<PointXYZIRT> nonground;

  ground.clear();
  nonground.clear();
  ground.header = temp_cloud.header;
  nonground.header = temp_cloud.header;
  _patchwork.segment_ground(temp_cloud, ground, nonground);

  RCLCPP_INFO(this->get_logger(), "Temp_cloud points %i", static_cast<int>(temp_cloud.size()));
  RCLCPP_INFO(this->get_logger(), "Ground points %i", static_cast<int>(ground.size()));
  RCLCPP_INFO(this->get_logger(), "Non ground points %i", static_cast<int>(nonground.size()));

  sensor_msgs::msg::PointCloud2 ground_msg;
  sensor_msgs::msg::PointCloud2 nonground_msg;
  
  pcl::toROSMsg(ground, ground_msg);
  pcl::toROSMsg(nonground, nonground_msg);
  RCLCPP_INFO(this -> get_logger(), "Header outgoing: %s", ground_msg.header.frame_id.c_str());

  _ground_publisher -> publish(ground_msg);
  _nonground_publisher -> publish(nonground_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
