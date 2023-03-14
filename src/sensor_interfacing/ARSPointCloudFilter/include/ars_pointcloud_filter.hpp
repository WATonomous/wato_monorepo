#ifndef ARS_POINTCLOUD_FILTER_HPP_
#define ARS_POINTCLOUD_FILTER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

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

  void reset_scan_states();

private:

  /**
  * @brief Pointfilter() filters an incoming radar message based on the parameters passed
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
  * @brief Variables below are used for all the filters
  */
  unsigned int default_timestamp_;
  
  typedef struct
  {
    unsigned int timestamp_;
    int packet_count_;
    bool publish_status_;
  } scan_params;

  /**
  * @brief Near & Far Scan Filter (Common Scan Filter)
  */
  scan_params near_scan_single_;
  scan_params far_scan_single_;
  radar_msgs::msg::RadarPacket buffer_packet_;


  /**
  * @brief Near Far Scan Filter (Double Buffer Algorithm)
  */
  int buffer_index_;
  std::array<radar_msgs::msg::RadarPacket, 2> near_far_buffer_packets_;
  std::array<scan_params, 2> near_scan_;
  std::array<scan_params, 2> far_scan_;

};

} // namespace filtering

#endif  // ARS_POINTCLOUD_FILTER_HPP_