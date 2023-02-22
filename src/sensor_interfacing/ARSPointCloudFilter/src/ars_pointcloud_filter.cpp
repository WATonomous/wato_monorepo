#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

// SOLUTION 1: Via Conditions and ROS parameters

namespace filtering
{

ARSPointCloudFilter::ARSPointCloudFilter()
{

}

// Point Filter Implementation

radar_msgs::msg::RadarPacket ARSPointCloudFilter::point_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
    double snr_threshold,
    double AzAng0_threshold,
    double range_threshold,
    double vrel_rad_threshold,
    double el_ang_threshold,
    double rcs_threshold)
    {
      radar_msgs::msg::RadarPacket filtered_ars;
      for (auto detection : unfiltered_ars->detections)
      {
        if(detection.snr < snr_threshold)
        {
          continue;
        }
        else if (detection.az_ang0 < AzAng0_threshold)
        {
          continue;
        }
        else if (detection.range < range_threshold)
        {
          continue;
        }
        else if (detection.vrel_rad < vrel_rad_threshold)
        {
          continue;
        }
        else if (detection.el_ang < el_ang_threshold)
        {
          continue;
        }
        else if (detection.rcs0 < rcs_threshold)
        {
          continue;
        }
        filtered_ars.detections.push_back(detection);
      }
      return filtered_ars;
    }


// Near Scan Filter Implementation
void ARSPointCloudFilter::near_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                          radar_msgs::msg::RadarPacket &buffer_packet, double vrel_rad_param, double el_ang_param,
                                          double rcs0_param, double snr_param, double range_param, double az_ang0_param, 
                                          std::pair<bool, radar_msgs::msg::RadarPacket> &publish_packet)
{
  if((msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5))
  {
    // Filter out far scan packets
      if(!buffer_packet.detections.empty())
      {
        // If timestamp is the same as buffer packet
        if(msg->timestamp == buffer_packet.timestamp)
        {
          // Filter out based on thresholds
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
                                                              
          // Append all detections to buffer 
          buffer_packet.detections.insert(buffer_packet.detections.end(), test_filtered_ars.detections.begin(), 
                                         test_filtered_ars.detections.end());
        }
        else
        {
          publish_packet.first = true;
          publish_packet.second = buffer_packet;
          
          // Filter incoming data
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
          
          // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
          buffer_packet = test_filtered_ars;
        }
    }
    else
    {
      // Filter incoming data based on parameters
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
      
      // Append filtered data to the buffer packet
      buffer_packet = test_filtered_ars;
    }
  }
}


// Far Scan Filter Implementation
void ARSPointCloudFilter::far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                          radar_msgs::msg::RadarPacket &buffer_packet, double vrel_rad_param, double el_ang_param,
                                          double rcs0_param, double snr_param, double range_param, double az_ang0_param, 
                                          std::pair<bool, radar_msgs::msg::RadarPacket> &publish_packet)
{
  if((msg->event_id == 1 || msg->event_id == 2))
  {
    // Filter out near scan packets
      if(!buffer_packet.detections.empty())
      {
        // If timestamp is the same as buffer packet
        if(msg->timestamp == buffer_packet.timestamp)
        {
          // Filter incoming data based on parameters
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
                                                              
          // Append all detections to buffer 
          buffer_packet.detections.insert(buffer_packet.detections.end(), test_filtered_ars.detections.begin(), 
                                         test_filtered_ars.detections.end());
        }
        else
        {
          publish_packet.first = true;
          publish_packet.second = buffer_packet;
          
          // Filter incoming data based on parameters
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
          
          // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
          buffer_packet = test_filtered_ars;
        }
    }
    else
    {
      // Filter incoming data based on parameters
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, snr_param, az_ang0_param, range_param, vrel_rad_param, el_ang_param, rcs0_param);
      
      // Append filtered data to the buffer packet
      buffer_packet = test_filtered_ars;
    }
  }
}


} // namespace filtering