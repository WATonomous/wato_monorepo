#include <chrono>
#include <memory>

#include "radar_pointcloud_filter_node.hpp"

PointCloudFilterNode::PointCloudFilterNode()
: Node("radar_point_cloud_filter")
{
    raw_left_sub_ = this->create_subscription<radar_msgs::msg::UnfilteredRadarLeft>(
    "unfilteredRadarLeft",
    std::bind(
      &PointCloudFilterNode::unfiltered_radar_left_callback, this,
      std::placeholders::_1));

    raw_right_sub_ = this->create_subscription<sample_msgs::msg::UnfilteredRadarRight>(
    "unfilteredRadarRight", ADVERTISING_FREQ,
    std::bind(
      &PointCloudFilterNode::unfiltered_radar_right_callback, this,
      std::placeholders::_1));

    raw_carla_left_sub_ = this->create_subscription<sample_msgs::msg::UnfilteredCarlaLeft>(
    "unfilteredCarlaLeft", ADVERTISING_FREQ,
    std::bind(
      &PointCloudFilterNode::unfiltered_carla_radar_left_callback, this,
      std::placeholders::_1));

    raw_carla_right_sub_ = this->create_subscription<sample_msgs::msg::UnfilteredCarlaRight>(
    "unfilteredCarlaRight", ADVERTISING_FREQ,
    std::bind(
      &PointCloudFilterNode::unfiltered_carla_radar_right_callback, this,
      std::placeholders::_1));
}
  
   void unfiltered_radar_right_callback(
    const radar_msgs::msg::UnfilteredRadarRight::SharedPtr msg)
    {
        //
    }

  
   void unfiltered_radar_left_callback(
    const radar_msgs::msg::UnfilteredRadarLeft::SharedPtr msg)
    {
        //
    }


  void unfiltered_carla_radar_left_callback(
    const radar_msgs::msg::UnfilteredCarlaLeft::SharedPtr msg)
    {
        //
    }


  void unfiltered_carla_radar_right_callback(
    const radar_msgs::msg::UnfilteredCarlaRight::SharedPtr msg)
    {
        //
    }

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
