#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"

// importing custom message types
#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"


namespace filtering
{
  
class ARSPointCloudFilter
{

public:

  ARSPointCloudFilter();

  radar_msgs::msg::RadarPacket point_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, 
  double snr_threshold,
  double AzAng0_threshold,
  double range_threshold,
  double vrel_rad_threshold,
  double el_ang_threshold,
  double rcs_threshold);
    
  typedef struct 
  {
    std::string scan_mode;
    double vrel_rad_param;
    double el_ang_param;
    double rcs0_param;
    double snr_param;
    double range_param;
    double az_ang0_param;
  } filter_parameters;
  
  bool near_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                        const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet);

  bool far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet);

  bool near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                            const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet);

private:

  // Create a buffer packet to hold detections from incoming messages (with the same timestamps)
  radar_msgs::msg::RadarPacket buffer_packet;

  // Create two buffer packets (for when both near and far scan mode filters are activated)
  radar_msgs::msg::RadarPacket buffer_packet_near_far_01;
  radar_msgs::msg::RadarPacket buffer_packet_near_far_02;

  // Number of detection packets
  int total_near_scan_packets = 0; // Would equal to 18 when full
  int total_far_scan_packets = 0; // Would equal to 12 when full
  int total_near_far = 30;

};

}

#endif  // ARS_POINTCLOUD_FILTER_HPP_
