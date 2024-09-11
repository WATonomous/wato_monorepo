#ifndef RADAR_VIS_HPP_
#define RADAR_VIS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "radar_msgs/msg/radar_detection.hpp"
#include "radar_msgs/msg/radar_packet.hpp"

namespace visualization {

class RadarVis {
 public:
  RadarVis();

  sensor_msgs::msg::PointCloud2 convert_packet_to_pointcloud(
      const radar_msgs::msg::RadarPacket::SharedPtr msg);

  visualization_msgs::msg::MarkerArray convert_packet_to_markers(
      const radar_msgs::msg::RadarPacket::SharedPtr msg);
};

}  // namespace visualization

#endif  // RADAR_VIS_HPP_
