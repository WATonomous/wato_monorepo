#ifndef RADAR_RVIZ_HPP_
#define RADAR_RVIZ_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

namespace visualization
{

class RadarRviz
{
public:
  RadarRviz();

  sensor_msgs::msg::PointCloud2 convert_packet_to_pointcloud(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);
};

}  // namespace visualization

#endif  // RADAR_RVIZ_HPP_
