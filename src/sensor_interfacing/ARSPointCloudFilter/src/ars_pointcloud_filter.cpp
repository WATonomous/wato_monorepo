#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

namespace filtering
{

ARSPointCloudFilter::ARSPointCloudFilter()
{
  packet_count_ = 0;
  packet_timestamp_ = -1;
  

  // make timestamps equal to 0
  buffer_index_ = 0;
  default_timestamp_ = 0;
  // near_timestamp_ = 0;
  // far_timestamp_ = 0;
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

// Implementation for checking event id and returning which scan it is (NEAR OR FAR)
scan_type ARSPointCloudFilter::check_scan_type(const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  if (msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5)
  {
    return NEAR;
  }
  else if (msg->event_id == 1 || msg->event_id == 2)
  {
    return FAR;
  }
}

void ARSPointCloudFilter::reset_scan_states(const int &buffer_index)
{
  near_scan_[buffer_index].timestamp_ = 0;
  near_scan_[buffer_index].publish_status_ = false;
  near_scan_[buffer_index].total_packet_count_= 0;

  far_scan_[buffer_index].timestamp_ = 0;
  far_scan_[buffer_index].publish_status_ = false;
  far_scan_[buffer_index].total_packet_count_= 0;

}

// Near Scan Filter Implementation
bool ARSPointCloudFilter::near_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg,
                                           const filter_parameters &parameters, 
                                           radar_msgs::msg::RadarPacket &publish_packet)
{
  if(msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5)
  {
    if(packet_timestamp_ == default_timestamp_)
    {
      packet_timestamp_ = msg->timestamp;
    }
        // Filter out detections based on given thresholds
    const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
          msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
          parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

    // If near packet is not full
    if(packet_count_ != 18 && msg->timestamp == packet_timestamp_)
    {
      // Append all detections to buffer
      buffer_packet_.detections.insert(buffer_packet_.detections.end(), 
                                      test_filtered_ars.detections.begin(), 
                                      test_filtered_ars.detections.end());
      packet_count_++;
    }
    else
    {
      /**
      Publish buffer packet since we have 18 packets.
      **/
     if(packet_count_!=18)
     {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Near packet is not full, size: %d ! \n ",packet_count_);
     }
      publish_packet = buffer_packet_;
      
      // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
      buffer_packet_ = test_filtered_ars;
      packet_count_ = 1;

      packet_timestamp_ = msg->timestamp;
      return true;
    }
  }
  return false;
}


// Far Scan Filter Implementation
bool ARSPointCloudFilter::far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                          const filter_parameters &parameters, 
                                          radar_msgs::msg::RadarPacket &publish_packet)
{
  if(msg->event_id == 1 || msg->event_id == 2)
  {
    if(packet_timestamp_ == default_timestamp_)
    {
      packet_timestamp_ = msg->timestamp;
    }

    // Filter out detections based on given thresholds
    const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
          msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
          parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

    // If near packet is not full
    if(packet_count_ != 12 && msg->timestamp == packet_timestamp_)
    {
      // Append all detections to buffer
      buffer_packet_.detections.insert(buffer_packet_.detections.end(), 
                                      test_filtered_ars.detections.begin(), 
                                      test_filtered_ars.detections.end());
      packet_count_++;
    }
    else
    {
      /**
      Publish buffer packet since we have 18 packets.
      **/
      if(packet_count_!=12)
      {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Far packet is not full, size: %d ! \n ", packet_count_);
      }
      publish_packet = buffer_packet_;

      // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
      buffer_packet_ = test_filtered_ars;
      packet_count_ = 1;

      packet_timestamp_ = msg->timestamp;
      return true;
    }
  }
  return false;
}


// Near + Far Scan Filter Implementation (Double Buffer Algorithm)
bool ARSPointCloudFilter::near_far_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg, 
                                               const filter_parameters &parameters,
                                               radar_msgs::msg::RadarPacket &publish_packet)
{
  // Variables to create on every call back (message can be near or far)
  scan_type incoming_scan_msg = check_scan_type(msg);

  int scan_capacity = (incoming_scan_msg == NEAR) ? 18 : 12;

  // Returns which scan parameters the program needs to work on (not a pointer)
  std::array<scan_params, 2>* scan = (incoming_scan_msg == NEAR) ? near_scan_ : far_scan_;

  const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
      msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
      parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

  if(msg->timestamp == default_timestamp_)
  {
    scan[buffer_index_].timestamp_ = msg->timestamp;
  }

  if(scan[buffer_index_].timestamp_ == msg->timestamp)
  {
    near_far_buffer_packets_[buffer_index_].detections.insert(near_far_buffer_packets_[buffer_index_].detections.end(),
                                                        test_filtered_ars.detections.begin(),
                                                        test_filtered_ars.detections.end());

    scan[buffer_index_].total_packet_count_++;
  }
  else
  {
    if(scan[buffer_index_].total_packet_count_ != scan_capacity)
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Packet is not full, Actual Size: %d, Expected: %d ! \n ", scan[buffer_index_].total_packet_count_, scan_capacity);
    }
    // ready to be published 
    scan[buffer_index_].publish_status_ = true;

    near_far_buffer_packets_[1 - buffer_index_].detections.insert(near_far_buffer_packets_[buffer_index_].detections.end(),
                                                    test_filtered_ars.detections.begin(),
                                                    test_filtered_ars.detections.end());
    scan[1 - buffer_index_].total_packet_count_++;
  }

  if(scan[buffer_index_].total_packet_count_ == scan_capacity)
  {
    scan[buffer_index_].publish_status_ == true;
  }

  if(near_scan_[buffer_index_].publish_status_ == true && far_scan_[buffer_index_].publish_status_ == true)
  {
    publish_packet = near_far_buffer_packets_[buffer_index_];
    
    near_far_buffer_packets_[buffer_index_].detections.clear();

    reset_scan_states(buffer_index_);

    buffer_index_ = 1 - buffer_index_;

    return true;
  }
  return false;


  // Edge Cases
  // 1. When there are more than 18 or 12 packets of the same timestamp being sent (rare)
  // 2. Data collections start in the middle instead of right in the beginning (0 packet count)





















  // // Check if its near
  // if (msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5)
  // {
  //   // Default case when no near packets are appended
  //   if (near_timestamp_ == default_timestamp_)
  //   {
  //     near_timestamp_ = msg->timestamp;
  //   }

  //   // Check if new message is part of the same scan
  //   if (msg->timestamp == near_timestamp_)
  //   {
  //     // Filter out detections based on given thresholds
  //     const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
  //           msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
  //           parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                          
  //     // Append all detections to buffer packet 1
  //     near_far_buffer_packets[buffer_index_].detections.insert(near_far_buffer_packets[buffer_index_].detections.end(),
  //                                                             test_filtered_ars.detections.begin(),
  //                                                             test_filtered_ars.detections.end());
  //     total_packet_count[buffer_index_]++;
  //   }

  //   // This means all 18 near scan packets are in buffer packet 1 (all with the same timestamps)
  //   else
  //   {
  //     next_near_timestamp_ = msg->timestamp;

  //     const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
  //           msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
  //           parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

  //     // Append to buffer packet 2
  //     near_far_buffer_packets[1 - buffer_index_].detections.insert(near_far_buffer_packets[buffer_index_].detections.end(),
  //                                                                 test_filtered_ars.detections.begin(),
  //                                                                 test_filtered_ars.detections.end());
  //     total_packet_count[1-buffer_index_]++;
  //   }
  // }

  // // Check if its far
  // else if (msg->event_id == 1 || msg->event_id == 2)
  // {
  //   // Default case when no far packets are appended
  //   if (far_timestamp_ == default_timestamp_)
  //   {
  //     far_timestamp_ = msg->timestamp;
  //   }

  //   // Check if new message is part of the same scan
  //   if (msg->timestamp == far_timestamp_)
  //   {
  //     // Filter out detections based on given thresholds
  //     const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
  //           msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
  //           parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);
                                                          
  //     // Append all detections to buffer packet 1
  //     near_far_buffer_packets[buffer_index_].detections.insert(near_far_buffer_packets[buffer_index_].detections.end(),
  //                                                             test_filtered_ars.detections.begin(),
  //                                                             test_filtered_ars.detections.end());
  //     total_packet_count[buffer_index_]++;
  //   }
    
  //   // This means that all 12 far scan packets are in buffer packet 1 (all with the same timestamps)
  //   else
  //   {
  //     next_far_timestamp_ = msg->timestamp;

  //     // Filter new far scan incoming data
  //     const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
  //           msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
  //           parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

  //     // Append to buffer packet 2
  //     near_far_buffer_packets[1 - buffer_index_].detections.insert(near_far_buffer_packets[buffer_index_].detections.end(),
  //                                                                 test_filtered_ars.detections.begin(),
  //                                                                 test_filtered_ars.detections.end());
  //     total_packet_count[1 - buffer_index_]++;
  //   }   

  //   if(total_packet_count[buffer_index_] == 30)
  //   {
  //     // This means that there are 30 packets in buffer packet 1 at this point

  //     // Publish buffer_packet
  //     publish_packet = near_far_buffer_packets[buffer_index_];
    
  //     // Change timestamps, next_near and next_far become the "new" timestamps to check if incoming messages are part of the same scan
  //     near_timestamp_ = next_near_timestamp_;
  //     far_timestamp_ = next_far_timestamp_;

  //     // Clear the buffer packet array
  //     near_far_buffer_packets[buffer_index_].detections.clear();
      
  //     total_packet_count[buffer_index_] = 0;
      
  //     // Flip index for next iteration
  //     buffer_index_ = 1 - buffer_index_;

  //     return true;

  //   }
  // }

  // return false;
}

} // namespace filtering