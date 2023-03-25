#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "radar_rviz_node.hpp"
#include "radar_rviz.hpp"

RadarRvizProcessorNode::RadarRvizProcessorNode()
: Node("radar_rviz")
{
//   raw_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
//     "processed",
//     1, std::bind(
//       &RadarRvizProcessorNode::process_radar_data_callback,
//       this, std::placeholders::_1));
}

// void RadarRvizProcessorNode::process_radar_data_callback(const radar_msgs::msg::RadarPacket msg)
// {


// }