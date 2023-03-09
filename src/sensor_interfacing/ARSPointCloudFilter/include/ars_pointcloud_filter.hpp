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

  enum scan_type 
  {
    NEAR,
    FAR
  };

  bool common_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters,
                       radar_msgs::msg::RadarPacket &publish_packet);

  bool near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters, 
                       radar_msgs::msg::RadarPacket &publish_packet);
  
  scan_type check_scan_type(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars);

  void reset_scan_states(const int &buffer_index);

private:

  // Filter message based on thresholds
  radar_msgs::msg::RadarPacket point_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
  double snr_threshold,
  double AzAng0_threshold,
  double range_threshold,
  double vrel_rad_threshold,
  double el_ang_threshold,
  double rcs_threshold);

  // For all Filters
  unsigned int default_timestamp_;


  // Create a struct with variables for near and far scans
  typedef struct
  {
    unsigned int timestamp_;
    int packet_count_;
    bool publish_status_;
  } scan_params;

  /*
  Near and far Scan filters
  */

  scan_params near_scan_single_;
  scan_params far_scan_single_;
  
  // Create a buffer packet to hold detections from incoming messages (with the same timestamps)
  radar_msgs::msg::RadarPacket buffer_packet_;


  /*
  Near + Far Scan Filter (Double buffer)
  */

  int buffer_index_;

  // Create two buffer packets
  std::array<radar_msgs::msg::RadarPacket, 2> near_far_buffer_packets_;

  // Create an array of structs for each scan type 
  std::array<scan_params, 2> near_scan_;
  std::array<scan_params, 2> far_scan_;

};

} // namespace filtering

#endif  // ARS_POINTCLOUD_FILTER_HPP_