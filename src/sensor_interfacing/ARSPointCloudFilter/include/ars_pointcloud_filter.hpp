#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include <string>
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
                       const filter_parameters &parameters,
                       radar_msgs::msg::RadarPacket &publish_packet);

  bool far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters, 
                       radar_msgs::msg::RadarPacket &publish_packet);

  bool near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters, 
                       radar_msgs::msg::RadarPacket &publish_packet);

private:

  // For near and far scan filters only
  int near_packet_count_;
  int far_packet_count_;
  unsigned int packet_timestamp_;
  
  // Create a buffer packet to hold detections from incoming messages (with the same timestamps)
  radar_msgs::msg::RadarPacket buffer_packet;



  // Create two buffer packets (for when both near and far scan mode filters are activated)
  radar_msgs::msg::RadarPacket near_far_buffer_packets[2];

  // Variables for double buffer
  int buffer_index;
  unsigned int default_timestamp;
  unsigned int near_timestamp;
  unsigned int far_timestamp;
  unsigned int next_near_timestamp;
  unsigned int next_far_timestamp;
};

} // namespace filtering

#endif  // ARS_POINTCLOUD_FILTER_HPP_
