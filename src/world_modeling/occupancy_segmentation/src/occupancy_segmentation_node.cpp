#include <chrono>
#include <memory>
#include <vector>

#include "occupancy_segmentation_node.hpp"

OccupancySegmentationNode::OccupancySegmentationNode()
: Node("occupancy_segmentation")
{
  pt_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/topic", ADVERTISING_FREQ,
    std::bind(
      &OccupancySegmentationNode::unfiltered_callback, this,
      std::placeholders::_1));
}
void OccupancySegmentationNode::unfiltered_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // aggregator_.add_raw_msg(msg);
  RCLCPP_INFO(
    this->get_logger(), "Raw Frequency(msg/s)");
}


void OccupancySegmentationNode::callback()
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
