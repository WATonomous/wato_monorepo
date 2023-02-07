#ifndef RADAR_POINTCLOUD_FILTER_HPP_
#define RADAR_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/RadarPacket.hpp"
#include "radar_msgs/msg/RadarDetection.hpp"

class ARSPointCloudFilter
{

public:
  ARSPointCloudFilter();

  void snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars_left,
                  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars_right);

  void radar_velocity_filter();

  void azimuth_angle_filter();

  void radar_cross_section_filter();


private:

};

