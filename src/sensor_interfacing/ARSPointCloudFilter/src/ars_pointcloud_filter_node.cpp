#include <chrono>
#include <memory>

#include "ars_pointcloud_filter_node.hpp"

ARSPointCloudFilterNode::ARSPointCloudFilterNode()
: Node("ars_point_cloud_filter")
{
  
  raw_left_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarLeft",
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback, this,
    std::placeholders::_1));

  raw_right_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarRight", ADVERTISING_FREQ,
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback, this,
    std::placeholders::_1));

}


void ARSPointCloudFilterNode :: unfiltered_ars_radar_right_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
    //messages from unfiltered right radar topic (ars)

}


void ARSPointCloudFilterNode :: unfiltered_ars_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
    //messages from unfiltered left radar topic (ars)
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ARSPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
