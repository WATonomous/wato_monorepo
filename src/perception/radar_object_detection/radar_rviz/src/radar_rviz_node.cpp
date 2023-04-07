#include <chrono>
#include <memory>

#include "radar_rviz_node.hpp"
#include "radar_rviz.hpp"

RadarRvizProcessorNode::RadarRvizProcessorNode()
: Node("radar_rviz")
{
  raw_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
    "processed",
    1, std::bind(
      &RadarRvizProcessorNode::process_radar_data_callback,
      this, std::placeholders::_1));
  raw_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visualization", 20);
}

void RadarRvizProcessorNode::process_radar_data_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 publish_packet_point_cloud;
  publish_packet_point_cloud = packet_to_rviz_processor_.convert_packet_to_pointcloud(msg);
  raw_pub_->publish(publish_packet_point_cloud);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing point cloud data to rviz \n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarRvizProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
