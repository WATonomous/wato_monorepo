#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "continental_pointcloud_filter_node.hpp"
#include "continental_pointcloud_filter.hpp"

namespace filtering
{

ContinentalPointCloudFilter::ContinentalPointCloudFilter()
{
  near_scan_single_.timestamp_ = 0;
  near_scan_single_.packet_count_ = 0;

  far_scan_single_.timestamp_ = 0;
  far_scan_single_.packet_count_ = 0;

  near_scan_[0].timestamp_ = 0;
  near_scan_[0].publish_status_ = false;
  near_scan_[0].packet_count_ = 0;

  near_scan_[1].timestamp_ = 0;
  near_scan_[1].publish_status_ = false;
  near_scan_[1].packet_count_ = 0;

  far_scan_[0].timestamp_ = 0;
  far_scan_[0].publish_status_ = false;
  far_scan_[0].packet_count_ = 0;

  far_scan_[1].timestamp_ = 0;
  far_scan_[1].publish_status_ = false;
  far_scan_[1].packet_count_ = 0;

  buffer_index_ = 0;
  default_timestamp_ = 0;
  near_scan_capacity_ = 18;
  far_scan_capacity_ = 12;
}

radar_msgs::msg::RadarPacket ContinentalPointCloudFilter::point_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_continental, double snr_threshold,
  double AzAng0_threshold, double range_threshold, double vrel_rad_threshold,
  double el_ang_threshold, double rcs_threshold)
{
  radar_msgs::msg::RadarPacket filtered_continental;
  for (auto detection : unfiltered_continental->detections) {
    if (detection.snr < snr_threshold) {
      continue;
    }
    if (detection.az_ang0 < AzAng0_threshold) {
      continue;
    }
    if (detection.range < range_threshold) {
      continue;
    }
    if (detection.vrel_rad < vrel_rad_threshold) {
      continue;
    }
    if (detection.el_ang < el_ang_threshold) {
      continue;
    }
    if (detection.rcs0 < rcs_threshold) {
      continue;
    }
    filtered_continental.detections.push_back(detection);
  }
  return filtered_continental;
}

ContinentalPointCloudFilter::scan_type ContinentalPointCloudFilter::check_scan_type(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  if (msg->event_id == 3 || msg->event_id == 4 || msg->event_id == 5) {
    return NEAR;
  } else if (msg->event_id == 1 || msg->event_id == 2) {
    return FAR;
  }
}

void ContinentalPointCloudFilter::reset_scan_states()
{
  near_scan_[buffer_index_].timestamp_ = 0;
  near_scan_[buffer_index_].publish_status_ = false;
  near_scan_[buffer_index_].packet_count_ = 0;

  far_scan_[buffer_index_].timestamp_ = 0;
  far_scan_[buffer_index_].publish_status_ = false;
  far_scan_[buffer_index_].packet_count_ = 0;
}

bool ContinentalPointCloudFilter::common_scan_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr msg,
  const filter_parameters & parameters,
  radar_msgs::msg::RadarPacket & publish_packet)
{
  scan_type incoming_scan_msg = check_scan_type(msg);

  int scan_capacity = (incoming_scan_msg == NEAR) ? near_scan_capacity_ : far_scan_capacity_;

  auto scan = (incoming_scan_msg == NEAR) ? &near_scan_single_ : &far_scan_single_;

  if ((incoming_scan_msg == NEAR && parameters.scan_mode == "near") ||
    (incoming_scan_msg == FAR && parameters.scan_mode == "far"))
  {
    if (scan->timestamp_ == default_timestamp_) {
      scan->timestamp_ = msg->timestamp;
    }

    const radar_msgs::msg::RadarPacket filtered_packet = point_filter(
      msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
      parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

    if (scan->packet_count_ != scan_capacity && msg->timestamp == scan->timestamp_) {
      buffer_packet_.timestamp = scan->timestamp_;
      buffer_packet_.detections.insert(
        buffer_packet_.detections.end(),
        filtered_packet.detections.begin(),
        filtered_packet.detections.end());
      scan->packet_count_++;

      if (scan->packet_count_ == scan_capacity) {
        publish_packet = buffer_packet_;
        buffer_packet_.detections.clear();
        scan->packet_count_ = 0;
        scan->timestamp_ = default_timestamp_;
        return true;
      }
    } else {
      // Special Case
      if (scan->packet_count_ != scan_capacity) {
        RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Packet is not full, size: %d! Packet Discarded.\n ", scan->packet_count_);
      }
      buffer_packet_ = filtered_packet;
      scan->packet_count_ = 1;
      scan->timestamp_ = msg->timestamp;
    }
  }
  return false;
}

bool ContinentalPointCloudFilter::near_far_scan_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr msg,
  const filter_parameters & parameters,
  radar_msgs::msg::RadarPacket & publish_packet)
{
  scan_type incoming_scan_msg = check_scan_type(msg);

  int scan_capacity = (incoming_scan_msg == NEAR) ? near_scan_capacity_ : far_scan_capacity_;

  auto scan = (incoming_scan_msg == NEAR) ? near_scan_.data() : far_scan_.data();

  const radar_msgs::msg::RadarPacket filtered_packet = point_filter(
    msg, parameters.snr_param, parameters.az_ang0_param, parameters.range_param,
    parameters.vrel_rad_param, parameters.el_ang_param, parameters.rcs0_param);

  if (scan[buffer_index_].timestamp_ == default_timestamp_) {
    scan[buffer_index_].timestamp_ = msg->timestamp;
  }

  if (scan[buffer_index_].timestamp_ == msg->timestamp) {
    // Special case
    if (incoming_scan_msg == FAR && near_scan_[buffer_index_].packet_count_ == 0) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Discarding far scan packet. \n");
      scan[buffer_index_].timestamp_ = default_timestamp_;
      return false;
    }
    near_far_buffer_packets_[buffer_index_].timestamp = scan[buffer_index_].timestamp_;

    near_far_buffer_packets_[buffer_index_].detections.insert(
      near_far_buffer_packets_[buffer_index_].detections.end(),
      filtered_packet.detections.begin(),
      filtered_packet.detections.end());

    scan[buffer_index_].packet_count_++;
  } else {
    if (scan[buffer_index_].packet_count_ != scan_capacity) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"), "Packet is not full, Actual Size: %d! \n",
        scan[buffer_index_].packet_count_);
    }

    scan[buffer_index_].publish_status_ = true;

    near_far_buffer_packets_[1 - buffer_index_].timestamp = scan[1 - buffer_index_].timestamp_;

    near_far_buffer_packets_[1 - buffer_index_].detections.insert(
      near_far_buffer_packets_[1 - buffer_index_].detections.end(),
      filtered_packet.detections.begin(),
      filtered_packet.detections.end());

    scan[1 - buffer_index_].packet_count_++;
  }

  if (scan[buffer_index_].packet_count_ == scan_capacity) {
    scan[buffer_index_].publish_status_ = true;
  }

  if (far_scan_[buffer_index_].publish_status_ == true) {
    // Special Case
    if (near_scan_[buffer_index_].packet_count_ != near_scan_capacity_ ||
      far_scan_[buffer_index_].packet_count_ != far_scan_capacity_)
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Incomplete total packet count! Not 30.\n");
      near_far_buffer_packets_[buffer_index_].detections.clear();
      reset_scan_states();
      buffer_index_ = 1 - buffer_index_;
      return false;
    }

    publish_packet = near_far_buffer_packets_[buffer_index_];
    near_far_buffer_packets_[buffer_index_].detections.clear();
    reset_scan_states();
    buffer_index_ = 1 - buffer_index_;
    return true;
  }
  return false;
}

}  // namespace filtering
