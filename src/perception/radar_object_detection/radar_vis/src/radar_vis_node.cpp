#include "radar_vis_node.hpp"

#include <memory>

#include "radar_vis.hpp"

RadarVisNode::RadarVisNode() : Node("radar_vis") {
  raw_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
      "processed", 1,
      std::bind(&RadarVisNode::process_radar_data_callback, this,
                std::placeholders::_1));
  raw_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "visualization", 20);
}

void RadarVisNode::process_radar_data_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg) {
  sensor_msgs::msg::PointCloud2 publish_packet_point_cloud;
  publish_packet_point_cloud =
      packet_converter_.convert_packet_to_pointcloud(msg);
  raw_pub_->publish(publish_packet_point_cloud);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarVisNode>());
  rclcpp::shutdown();
  return 0;
}
