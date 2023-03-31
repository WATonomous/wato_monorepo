#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "radar_rviz_node.hpp"
#include "radar_rviz.hpp"

namespace processing
{

RadarRvizProcessor::RadarRvizProcessor()
{}

sensor_msgs::msg::PointCloud2 RadarRvizProcessor::convert_packet_to_pointcloud(
    const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 packet;

    return packet;

}

}

