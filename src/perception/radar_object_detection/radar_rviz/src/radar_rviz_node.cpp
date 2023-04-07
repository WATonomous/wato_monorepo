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
  // Temporary point cloud data
  sensor_msgs::msg::PointCloud2 publish_packet_point_cloud;
  publish_packet_point_cloud = packet_to_rviz_processor_.convert_packet_to_pointcloud(msg);
  raw_pub_->publish(publish_packet_point_cloud);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing point cloud data to rviz \n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();
  radar_msgs::msg::RadarDetection msg_detection;

  msg->event_id = 3;
  msg->timestamp = 2;


  auto test_node = std::make_shared<RadarRvizProcessorNode>();
  rclcpp::Rate loop_rate(2000);

  while(rclcpp::ok())
  {
    msg_detection.pos_x = 1;
    msg_detection.pos_y = 1;
    msg_detection.pos_z = 1;
    msg_detection.rcs0 = 1;
    msg->detections.push_back(msg_detection);

    RCLCPP_INFO(test_node->get_logger(), "%s\n", "Sending data to callback.");
    test_node->process_radar_data_callback(msg);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
