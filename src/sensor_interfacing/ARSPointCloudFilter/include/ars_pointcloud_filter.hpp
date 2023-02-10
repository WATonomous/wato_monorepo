#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

// importing custom message types
#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

class ARSPointCloudFilter
{
public:
  ARSPointCloudFilter();

  radar_msgs::msg::RadarPacket snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold);

  radar_msgs::msg::RadarPacket azimuth_angle_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                            double AzAng0_threshold, double AzAng1_threshold);

  radar_msgs::msg::RadarPacket range_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double range_threshold);

};

#endif  // ARS_POINTCLOUD_FILTER_HPP_
