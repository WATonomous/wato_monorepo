#ifndef RADAR_POINTCLOUD_FILTER_HPP_
#define RADAR_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/UnfilteredRadarLeftPacket.hpp"
#include "radar_msgs/msg/UnfilteredRadarRightPacket.hpp"
#include "radar_msgs/msg/UnfilteredCarlaLeftPacket.hpp"
#include "radar_msgs/msg/UnfilteredCarlaRightPacket.hpp"
#include "radar_msgs/msg/RadarDetection.hpp



//SAMPLE FUNCTION

class PointCloudFilter
{

public:
  void snr_filter();

}

