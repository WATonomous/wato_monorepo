#ifndef RADAR_POINTCLOUD_FILTER_HPP_
#define RADAR_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

//importing custom message types
#include "radar_msgs/msg/RadarPacket.hpp"
#include "radar_msgs/msg/RadarDetection.hpp"

class ARSPointCloudFilter
{

public:
  ARSPointCloudFilter();

  void snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, float snr_threshold);

  void azimuth_angle_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                            float AzAng0_threshold, float AzAng1_threshold);

  void range_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, float range_threshold);

private:

};

