#ifndef CONTINENTAL_POINTCLOUD_FILTER_HPP_
#define CONTINENTAL_POINTCLOUD_FILTER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

namespace filtering
{

/**
* @brief The ContinentalPointCloudFilter is responsible for filtering, and publishing radar detections
*    coming from the ARS430 radar sensor in the Radar ROS2 pipeline. These incoming
*    detections are first organized and transformed into custom ROS messages/packets prior
*    to being sent into this node. Note each message has detections of two types.
*    These detections can be from a near scan or a far scan.
*
*    The internal logic for this node is divided into three sections or "modes".
*
*    Mode 1. The node recieves, filters and publishes only NEAR scan radar detections.
*    These are packets with the event id 3, 4 or 5.
*
*    Mode 2. The node recieves, filters and publishes only FAR scan radar detections
*    These are packets with the event id 1 or 2.
*
*    Mode 3. The node recieves, filters and publishes BOTH NEAR and FAR scan radar detections.
*    In this case, it is much more complex to handle detections from both NEAR and FAR scan
*    especially if they are also from two seperate scans. To solve this, the node uses the double
*    buffer algorithm.
*/
class ContinentalPointCloudFilter
{
public:
  ContinentalPointCloudFilter();
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
  * @brief The following is the internal logic of the common_scan_filter for near and far scan modes.
  *    This filter works by checking the timestamps of incoming messages. If messages share the same
  *    timestamp, this means that they are part of the same scan. Therefore, all the detections from
  *    that incoming message are filtered and appended to a buffer packet. This process continues
  *    until the packet count is at maxiumum capacity or when there is a timestamp change indicating
  *    that a new scan has begun and the old buffer packet is ready to be published.
  *    Special case to consider: If the scan starts in the middle such that we don't have a
  *    complete number of packets (18 or 12) collected, then the code discards these detections
  *    and starts from scratch with a new scan.
  */
  bool common_scan_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_continental,
    const filter_parameters & parameters,
    radar_msgs::msg::RadarPacket & publish_packet);

  /**
  * @brief The following is the internal logic of the near_far_scan_filter
  *    for the "nearfar" mode. This is when the double buffer algorithm is utilized.
  *    Please refer to the figure in the WATO drive for more information
  *    on the algorithm.
  *    Special cases to consider:
  *    1. If the scan started in the middle such that there is an incomplete packets of
  *       near scans but a full number of far scans. This means that we will have
  *       less than 30 packets when publishing. Therefore, the program discards this
  *       incomplete packet and starts from scratch.
  *    2. If the scan started in the middle, and we are only receiving far scan packets
  *       for that scan (no near scan packets were collected). Then the program discards
  *       each far scan packet that is recieved from that scan.
  */
  bool near_far_scan_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_continental,
    const filter_parameters & parameters,
    radar_msgs::msg::RadarPacket & publish_packet);

  /**
  * @brief Checks the Event ID of a message and returns which scan it is (NEAR OR FAR).
  */
  scan_type check_scan_type(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_continental);

  /**
  * @brief Resets all scan states (timestamp, packet count, and publish status).
  */
  void reset_scan_states();

private:
  /**
  * @brief Pointfilter() filters an incoming radar packet based on set thresholds.
  */
  radar_msgs::msg::RadarPacket point_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_continental,
    double snr_threshold,
    double AzAng0_threshold,
    double range_threshold,
    double vrel_rad_threshold,
    double el_ang_threshold,
    double rcs_threshold);

  /**
  * @brief Variables below are used for all the filters (Common scan filter & NearFarScan Filter).
  */
  unsigned int default_timestamp_;
  typedef struct
  {
    unsigned int timestamp_;
    int packet_count_;
    bool publish_status_;
  } scan_params;

  /**
  * @brief Variables only used for common scan filter.
  */
  scan_params near_scan_single_;
  scan_params far_scan_single_;
  radar_msgs::msg::RadarPacket buffer_packet_;

  /**
  * @brief Variables only used for Near Far Scan Filter (Double Buffer Algorithm).
  */
  int buffer_index_;
  std::array<radar_msgs::msg::RadarPacket, 2> near_far_buffer_packets_;
  std::array<scan_params, 2> near_scan_;
  std::array<scan_params, 2> far_scan_;
};

}  // namespace filtering

#endif  // CONTINENTAL_POINTCLOUD_FILTER_HPP_
