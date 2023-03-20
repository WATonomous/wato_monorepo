#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

namespace filtering
{

/**
* @brief The ARSPointCloudFilter is responsible for filtering and publishing radar detections 
*        coming through custom ROS messages/packets. These packets can be composed of near scan
*        or far scan detections. 
*        There are various modes that can be set for the node.
*        Mode 1. Recieves, filters and publishes only NEAR scan radar detections (Event ID 3, 4 or 5).
*        Mode 2. Recieves, filters and publishes only FAR scan radar detections (Event ID 1, or 2).
*        Mode 3. Recieves, filters and publishes BOTH near and far scan radar detections. In this case, 
*        the implementation below follows the double buffer algorithm which allows the node
*        to not only handle incoming packets from different scans but also filter and publish them
*        accordingly (Refer to google drawing on the WATO Drive for more info)
*                   
*/
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

  /**
  * @brief A common filter for near and far scan modes - EXPLAIN
  */
  bool common_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters,
                       radar_msgs::msg::RadarPacket &publish_packet);

  /**
  * @brief Near + Far Scan Filter Implementation (Double Buffer Algorithm) - EXPLAIN
  */
  bool near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                       const filter_parameters &parameters, 
                       radar_msgs::msg::RadarPacket &publish_packet);
  
  /**
  * @brief Checks Event ID and returns which scan it is (NEAR OR FAR)
  */
  scan_type check_scan_type(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars);

  /**
  * @brief Resets all scan states
  */
  void reset_scan_states();

private:

  /**
  * @brief Pointfilter() filters an incoming radar packet based on set thresholds
  */
  radar_msgs::msg::RadarPacket point_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
  double snr_threshold,
  double AzAng0_threshold,
  double range_threshold,
  double vrel_rad_threshold,
  double el_ang_threshold,
  double rcs_threshold);

  /**
  * @brief Variables below are used for all the filters (Common scan filter & NearFarScan Filter)
  */
  unsigned int default_timestamp_;
  
  typedef struct
  {
    unsigned int timestamp_;
    int packet_count_;
    bool publish_status_;
  } scan_params;

  /**
  * @brief Variables only used for common scan filter
  */
  scan_params near_scan_single_;
  scan_params far_scan_single_;
  radar_msgs::msg::RadarPacket buffer_packet_;

  /**
  * @brief Variables only used for Near Far Scan Filter (Double Buffer Algorithm)
  */
  int buffer_index_;
  std::array<radar_msgs::msg::RadarPacket, 2> near_far_buffer_packets_;
  std::array<scan_params, 2> near_scan_;
  std::array<scan_params, 2> far_scan_;

};

} // namespace filtering

#endif  // ARS_POINTCLOUD_FILTER_HPP_