#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

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
bool ARSPointCloudFilter::near_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                           const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet)
{
  if((msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5))
  {
      // Filter out far scan packets
      if(!buffer_packet.detections.empty())
      {
        // If timestamp is the same as buffer packet
        if(msg->timestamp == buffer_packet.timestamp)
        {
          // Filter out detections based on given thresholds
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                              
          // Append all detections to buffer 
          buffer_packet.detections.insert(buffer_packet.detections.end(), test_filtered_ars.detections.begin(), 
                                         test_filtered_ars.detections.end());
          return false;
          
        }
        else
        {
          /**
          Publish buffer packet since incoming message now has a different timestamp. 
          This means that the buffer packet is full with detections from 18 packets of the same timestamp.
          **/
          publish_packet = buffer_packet;
          
          // Filter new incoming data
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
          
          // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
          buffer_packet = test_filtered_ars;

          return true;
        }
    }
    else
    {
      // Filter incoming data based on parameters
      const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
            msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
            parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
      
      // Append filtered data to the buffer packet
      buffer_packet = test_filtered_ars;

      return false;
    }
  }

  return false;
}


// Far Scan Filter Implementation
bool ARSPointCloudFilter::far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                          const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet)
{
  if((msg->event_id == 1 || msg->event_id == 2))
  {
    // Filter out near scan packets
      if(!buffer_packet.detections.empty())
      {
        // If timestamp is the same as buffer packet
        if(msg->timestamp == buffer_packet.timestamp)
        {
          // Filter incoming data based on given thresholds
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                              
          // Append all detections to buffer 
          buffer_packet.detections.insert(buffer_packet.detections.end(), test_filtered_ars.detections.begin(), 
                                         test_filtered_ars.detections.end());

          return false;
        }
        else
        {
          /**
          Publish buffer packet since incoming message now has a different timestamp. 
          This means that the buffer packet is full with detections from 18 packets of the same timestamp.
          **/
          publish_packet = buffer_packet;
          
          // // Filter new incoming data
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
          
          // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
          buffer_packet = test_filtered_ars;

          return true;
        }
    }
    else
    {
      // Filter incoming data based on parameters
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
      
      // Append filtered data to the buffer packet
      buffer_packet = test_filtered_ars;

      return false;
    }
  }

  return false;
}





// Near + Far Scan Filter Implementation (Double Buffer Algorithm)
bool ARSPointCloudFilter::near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                          const filter_parameters &parameters, radar_msgs::msg::RadarPacket &publish_packet)
{
  // Scan 1 (same timestamp for 18 near scan packets and a slightly different timestamp for 12 far scan packets)
  // But they should be appended to the same packet (published in 30)

  // Pseudocode

  // Save near and far scan timestamps as member variables

  if event id == near and msg timestamp != near timestamp member variable 
     then update member timestamp, make scan1_near == false, scan2_near == true

  else keep the same timestamp for near, make scan1_near = true, scan2_near = false

  if event id is far and msg timestamp != far timestamp member variable 
     then update member timestamp, make scan1_far == false, scan2_far = true

  else keep the same timestamp for far, make scan1_far = true, scan2_far = false

  if (msg timetamp  == near or far member timestamps)
  {
      if msg timestamp == near timestamp and scan1_near == true and scan2_near = false, 
      {
      append detections to buffer packet 1 
      total_near_packets_count++ 
      }

      // Means near scans are full in buffer packet 1 
      else 
      {
      Append detections to buffer packet 2 
      }

      if msg timestamp == far timestamp and scan1_far == true, scan2_far = false,
      {
      Append detections to buffer packet 1
      total_far_packets_count++
      }

      // Means far scans are full in buffer packet 1 
      else 
      {
      Append to buffer packet 2 (this means that we have 30 packets in buffer packet 1)
      Publish buffer packet 1, (both member timestamps have already been updated at this point for scan 2) 
      Reset packet counters
      }
  }

}







} // namespace filtering