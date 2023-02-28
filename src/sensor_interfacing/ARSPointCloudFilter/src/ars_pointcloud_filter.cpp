#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

namespace filtering
{

ARSPointCloudFilter::ARSPointCloudFilter()
{
  buffer_index = 0;
  default_timestamp = -1;
  near_timestamp = -1;
  far_timestamp = -1;
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
                                           const filter_parameters &parameters, 
                                           radar_msgs::msg::RadarPacket &publish_packet)
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
          buffer_packet.detections.insert(buffer_packet.detections.end(), 
                                          test_filtered_ars.detections.begin(), 
                                          test_filtered_ars.detections.end());
          return false;
        }

        else
        {
          /**
          Publish buffer packet since the incoming message will now have a different timestamp.
          This means that the buffer packet is full with detections from 18 packets
          of the same timestamp.
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
      // Filter incoming data based on given thresholds
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
                                          const filter_parameters &parameters, 
                                          radar_msgs::msg::RadarPacket &publish_packet)
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
                                                              
          // Append all detections to buffer packet
          buffer_packet.detections.insert(buffer_packet.detections.end(), 
                                          test_filtered_ars.detections.begin(), 
                                          test_filtered_ars.detections.end());

          return false;
        }

        else
        {
          /**
          Publish buffer packet since the incoming message will now have a different timestamp. 
          This means that the buffer packet is full of detections from 18 packets
          of the same timestamp.
          **/
          publish_packet = buffer_packet;
          
          // Filter new incoming data
          const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
                msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
                parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
          
          // Replace existing data in buffer packet with new data (new timestamp) after filtering
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
                                          const filter_parameters &parameters,
                                          radar_msgs::msg::RadarPacket &publish_packet)
{
  // Check if its near
  if (msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5)
  {
    // Default case when no near packets are appended
    if (near_timestamp == default_timestamp)
    {
      near_timestamp = msg->timestamp;
    }

    // Check if new message is part of the same scan
    if (msg->timestamp == near_timestamp)
    {
      // Filter out detections based on given thresholds
      const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
            msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
            parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                          
      // Append all detections to buffer packet 1
      near_far_buffer_packets[buffer_index].detections.insert(near_far_buffer_packets[buffer_index].detections.end(),
                                                              test_filtered_ars.detections.begin(),
                                                              test_filtered_ars.detections.end());
      
      return false;

    }

    // This means all 18 near scan packets are in buffer packet 1 (all with the same timestamps)
    else
    {
      next_near_timestamp = msg->timestamp;

      const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
            msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
            parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

      // Append to buffer packet 2
      near_far_buffer_packets[1 - buffer_index].detections.insert(near_far_buffer_packets[buffer_index].detections.end(),
                                                                  test_filtered_ars.detections.begin(),
                                                                  test_filtered_ars.detections.end());
      
      return false;

    }
  }

  // Check if its far
  if (msg->event_id == 1 || msg->event_id == 2)
  {
    // Default case when no far packets are appended
    if (far_timestamp == default_timestamp)
    {
      far_timestamp = msg->timestamp;
    }

    // Check if new message is part of the same scan
    if (msg->timestamp == far_timestamp)
    {
      // Filter out detections based on given thresholds
      const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
            msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
            parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                          
      // Append all detections to buffer packet 1
      near_far_buffer_packets[buffer_index].detections.insert(near_far_buffer_packets[buffer_index].detections.end(),
                                                              test_filtered_ars.detections.begin(),
                                                              test_filtered_ars.detections.end());
      return false;

    }
    
    // This means that all 12 far scan packets are in buffer packet 1 (all with the same timestamps)
    else
    {
      next_far_timestamp = msg->timestamp;

      // This also means that there are 30 packets in buffer packet 1 at this point

      // Publish buffer_packet
      publish_packet = near_far_buffer_packets[buffer_index];

      // Filter new far scan incoming data
      const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
            msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
            parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

      // Append to buffer packet 2
      near_far_buffer_packets[1 - buffer_index].detections.insert(near_far_buffer_packets[buffer_index].detections.end(),
                                                                  test_filtered_ars.detections.begin(),
                                                                  test_filtered_ars.detections.end());

      // Change timestamps, next_near and next_far become the "new" timestamps to check if incoming messages are part of the same scan
      near_timestamp = next_near_timestamp;
      far_timestamp = next_far_timestamp;

      // Clear the buffer packet array
      near_far_buffer_packets[buffer_index].detections.clear();

      // Flip index for next iteration
      buffer_index = 1 - buffer_index;

      return true;
    }

  }
  
  return false;
}

} // namespace filtering