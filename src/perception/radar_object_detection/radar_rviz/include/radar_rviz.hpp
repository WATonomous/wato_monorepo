#ifndef RADAR_RVIZ_HPP_
#define RADAR_RVIZ_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

// #include "radar_msgs/msg/radar_packet.hpp"
// #include "radar_msgs/msg/radar_detection.hpp"

namespace processing
{
class RadarRvizProcessor
{
public:
  RadarRvizProcessor();

private:
  // bool convert_packet_to_pointcloud(
  //   const radar_msgs::msg:RadarPacket::SharedPtr msg);
};

}  // namespace processing

#endif  // RADAR_RVIZ_HPP_