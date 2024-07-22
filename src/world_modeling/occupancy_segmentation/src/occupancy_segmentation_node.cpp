#include <memory>
#include <iostream>
#include "occupancy_segmentation_node.hpp"

OccupancySegmentationNode::OccupancySegmentationNode() : Node("occupancy_segmentation") {
  // _subscriber =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "topic", 10, std::bind(&OccupancySegmentationNode::subscription_callback, this));

}

// void OccupancySegmentationNode::subscription_callback(const  sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud){
//   // pcl::PointCloud<pcl::PointXYZ> temp_cloud;
//   // pcl::fromROSMsg(*lidar_cloud, temp_cloud);

//   // pcl::PointCloud<pcl::PointXYZ> ground;
//   // pcl::PointCloud<pcl::PointXYZ> nonground;
//   // _patchwork.segment_ground(temp_cloud, ground, nonground);

//   // sensor_msgs::msg::PointCloud2::SharedPtr ground_msg;
//   // sensor_msgs::msg::PointCloud2::SharedPtr nonground_msg;
//   // pcl::toROSMsg(ground, *ground_msg);
//   // pcl::toROSMsg(nonground, *nonground_msg);

// }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
