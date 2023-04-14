
#include <chrono>
#include <memory>

#include "radar_centerfusion_node.hpp"
#include "radar_centerfusion.hpp"

RadarCenterFusionNode::RadarCenterFusionNode()
: Node("radar_centerfusion")
{
  raw_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
    "radar_data",
    1, std::bind(
      &RadarCenterFusionNode::radar_data_callback,
      this, std::placeholders::_1));
  raw_pub_ = this->create_publisher<radar_msgs::msg::RadarPacket>("centerfusion_inference", 20);
}

void RadarCenterFusionNode::radar_data_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarCenterFusionNode>());
  rclcpp::shutdown();
  return 0;
}