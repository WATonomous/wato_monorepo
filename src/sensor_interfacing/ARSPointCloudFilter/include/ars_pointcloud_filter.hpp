#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

// importing custom message types
#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "ars_pointcloud_filter_node.hpp"

namespace filtering
{
  
class ARSPointCloudFilter
{
public:
  ARSPointCloudFilter();

  // point_filter - Solution 1
  radar_msgs::msg::RadarPacket point_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, 
    double snr_threshold,
    double AzAng0_threshold,
    double range_threshold,
    double vrel_rad_threshold,
    double el_ang_threshold,
    double rcs_threshold);
  
  void near_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                        radar_msgs::msg::RadarPacket &buffer_packet, double vrel_rad_param, double el_ang_param,
                        double rcs0_param, double snr_param, double range_param, double az_ang0_param);

  // void far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
  //                     radar_msgs::msg::RadarPacket &buffer_packet, struct ARSPointCloudFilterNode::filter_parameters parameters);







  // radar_msgs::msg::RadarPacket snr_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold);

  // radar_msgs::msg::RadarPacket azimuth_angle_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double AzAng0_threshold);

  // radar_msgs::msg::RadarPacket range_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double range_threshold);

  // radar_msgs::msg::RadarPacket vrel_rad_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double vrel_rad_threshold);

  // radar_msgs::msg::RadarPacket el_ang_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double el_ang_threshold);

  // radar_msgs::msg::RadarPacket rcs_filter(
  //   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double rcs_threshold);
};
}
#endif  // ARS_POINTCLOUD_FILTER_HPP_
