#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

namespace filtering
{

ARSPointCloudFilter::ARSPointCloudFilter()
{ 
  near_scan_single_.timestamp_ = 0;
  near_scan_single_.packet_count_= 0;

  far_scan_single_.timestamp_ = 0;
  far_scan_single_.packet_count_= 0;

  near_scan_[0].timestamp_ = 0;
  near_scan_[0].publish_status_ = false;
  near_scan_[0].packet_count_= 0;

  near_scan_[1].timestamp_ = 0;
  near_scan_[1].publish_status_ = false;
  near_scan_[1].packet_count_= 0;

  far_scan_[0].timestamp_ = 0;
  far_scan_[0].publish_status_ = false;
  far_scan_[0].packet_count_= 0;

  far_scan_[1].timestamp_ = 0;
  far_scan_[1].publish_status_ = false;
  far_scan_[1].packet_count_= 0;

  buffer_index_ = 0;
  default_timestamp_ = 0;
  
}

// Point Filter Implementation

radar_msgs::msg::RadarPacket ARSPointCloudFilter::point_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold,
    double AzAng0_threshold, double range_threshold, double vrel_rad_threshold, 
    double el_ang_threshold, double rcs_threshold)
{
  radar_msgs::msg::RadarPacket filtered_ars;
  for (auto detection : unfiltered_ars->detections)
  {
    if(detection.snr < snr_threshold)
    {
      continue;
    }
    if (detection.az_ang0 < AzAng0_threshold)
    {
      continue;
    }
    if (detection.range < range_threshold)
    {
      continue;
    }
    if (detection.vrel_rad < vrel_rad_threshold)
    {
      continue;
    }
    if (detection.el_ang < el_ang_threshold)
    {
      continue;
    }
    if (detection.rcs0 < rcs_threshold)
    {
      continue;
    }
    filtered_ars.detections.push_back(detection);
  }
  return filtered_ars;
}

// Checks Event ID and returns which scan it is (NEAR OR FAR)
ARSPointCloudFilter::scan_type ARSPointCloudFilter::check_scan_type(const radar_msgs::msg::RadarPacket::SharedPtr msg)
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

void ARSPointCloudFilter::reset_scan_states()
{
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Reseting all states! \n ");
  near_scan_[buffer_index_].timestamp_ = 0;
  near_scan_[buffer_index_].publish_status_ = false;
  near_scan_[buffer_index_].packet_count_= 0;

  far_scan_[buffer_index_].timestamp_ = 0;
  far_scan_[buffer_index_].publish_status_ = false;
  far_scan_[buffer_index_].packet_count_= 0;

}

// A common filter for near and far scan modes
bool ARSPointCloudFilter::common_scan_filter(const radar_msgs::msg::RadarPacket::SharedPtr msg,
                                           const filter_parameters &parameters, 
                                           radar_msgs::msg::RadarPacket &publish_packet)
{
  // Variables to create on every callback (message can be near or far)
  scan_type incoming_scan_msg = check_scan_type(msg);

  int scan_capacity = (incoming_scan_msg == NEAR) ? 18 : 12;

  // Returns which scan parameters the program needs to work on
  auto scan = (incoming_scan_msg == NEAR) ? &near_scan_single_ : &far_scan_single_;

  if ((incoming_scan_msg == NEAR && parameters.scan_mode == "near") || (incoming_scan_msg == FAR && parameters.scan_mode == "far"))
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Initial Packet Timestamp %d \n ",scan->timestamp_);
    
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Initial Packet Size %d \n ",scan->packet_count_);
    
    if(scan->timestamp_ == default_timestamp_)
    {
      scan->timestamp_ = msg->timestamp;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Checking for Default Timestamp \n ");
    }
    
    // Filter out detections based on given thresholds (PASS IN ONLY PARAMETERS)
    const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
          msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
          parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Filtering the message \n ");

    // If near/far packet is not full
    if(scan->packet_count_ != scan_capacity && msg->timestamp == scan->timestamp_)
    {
      // Update buffer packet timestamp
      buffer_packet_.timestamp = scan->timestamp_;

      // Append all detections to buffer
      buffer_packet_.detections.insert(buffer_packet_.detections.end(), 
                                      test_filtered_ars.detections.begin(), 
                                      test_filtered_ars.detections.end());
      scan->packet_count_++;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Appending to buffer packet, size: %d\n ",scan->packet_count_);

      if(scan->packet_count_ == scan_capacity)
      {
        // Publish buffer packet
        publish_packet = buffer_packet_;
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Publishing Packet, size: %d\n ",scan->packet_count_);

        buffer_packet_.detections.clear();

        scan->packet_count_ = 0;

        scan->timestamp_ = default_timestamp_;
        return true;
      }
      
    }
    else
    {
      if(scan->packet_count_!= scan_capacity)
      {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Packet is not full, size: %d ! \n ", scan->packet_count_);
      }

      // Publish buffer packet
      publish_packet = buffer_packet_;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Publishing Packet, size: %d\n ",scan->packet_count_);
      
      // Replace existing data in buffer packet with new incoming data (new timestamp) after filtering
      buffer_packet_ = test_filtered_ars;
      scan->packet_count_ = 1;

      scan->timestamp_ = msg->timestamp;
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
  // Variables to create on every callback (message can be near or far)
  scan_type incoming_scan_msg = check_scan_type(msg);

  int scan_capacity = (incoming_scan_msg == NEAR) ? 18 : 12;

  // Returns which scan parameters the program needs to work on
  auto scan = (incoming_scan_msg == NEAR) ? near_scan_.data() : far_scan_.data();

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Filtering the message \n ");

  const radar_msgs::msg::RadarPacket test_filtered_ars = point_filter(
      msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
      parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);


  if(scan[buffer_index_].timestamp_ == default_timestamp_)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Buffer Index: %d\n ", buffer_index_);
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Comparing Scan buffer Timestamp with default timestamp, Scan Buffer Timestamp: %d\n ", scan[buffer_index_].timestamp_);
    scan[buffer_index_].timestamp_ = msg->timestamp;
  }

  if(scan[buffer_index_].timestamp_ == msg->timestamp)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Checking for incoming msg timestamp \n ");
    
    near_far_buffer_packets_[buffer_index_].timestamp = scan[buffer_index_].timestamp_;

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Appending detections to packet, Buffer Index: %d\n ", buffer_index_);

    near_far_buffer_packets_[buffer_index_].detections.insert(near_far_buffer_packets_[buffer_index_].detections.end(),
                                                              test_filtered_ars.detections.begin(),
                                                              test_filtered_ars.detections.end());
    scan[buffer_index_].packet_count_++;
  }
  else
  {
    if(scan[buffer_index_].packet_count_ != scan_capacity)
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Packet is not full, Actual Size: %d! \n ", scan[buffer_index_].packet_count_);
    }

    scan[buffer_index_].publish_status_ = true;

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Appending detections to other packet, Buffer Index: %d! \n ", 1 - buffer_index_);

    near_far_buffer_packets_[1 - buffer_index_].timestamp = scan[1 - buffer_index_].timestamp_;

    near_far_buffer_packets_[1 - buffer_index_].detections.insert(near_far_buffer_packets_[1 - buffer_index_].detections.end(),
                                                                  test_filtered_ars.detections.begin(),
                                                                  test_filtered_ars.detections.end());
    scan[1 - buffer_index_].packet_count_++;
  }

  if(scan[buffer_index_].packet_count_ == scan_capacity)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Checking for scan capacity: %d  \n ", scan_capacity);
    scan[buffer_index_].publish_status_ = true;
  }

  // Removed "near_scan_[buffer_index_].publish_status_ == true" from if condition
  // then if this is true, then we know in general that packet is done collecting data from that particular scan (whether its the full 30 or not)
  if(far_scan_[buffer_index_].publish_status_ == true)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Checking if both packets are ready to be published! \n ");
    
    publish_packet = near_far_buffer_packets_[buffer_index_];

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Publishing packet, Detections: %d \n ", publish_packet.detections.size());
    
    near_far_buffer_packets_[buffer_index_].detections.clear();

    reset_scan_states();

    buffer_index_ = 1 - buffer_index_;

    return true;
  }
  return false;
}

} // namespace filtering