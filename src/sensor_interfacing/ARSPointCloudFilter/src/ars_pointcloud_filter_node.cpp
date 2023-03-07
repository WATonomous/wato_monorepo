#include <chrono>
#include <memory>

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

ARSPointCloudFilterNode::ARSPointCloudFilterNode(): Node("ars_point_cloud_filter")
{
  // Default values already declared in ars_radar_params.yaml

  this->declare_parameter("filter_mode");
  this->declare_parameter("scan_mode");
  this->declare_parameter("vrel_rad");
  this->declare_parameter("el_ang");
  this->declare_parameter("rcs0");
  this->declare_parameter("snr");
  this->declare_parameter("range");
  this->declare_parameter("az_ang0");

  parameters.scan_mode = this->get_parameter("scan_mode").as_string();
  parameters.vrel_rad_param = this->get_parameter("vrel_rad").as_double();
  parameters.el_ang_param = this->get_parameter("el_ang").as_double();
  parameters.rcs0_param = this->get_parameter("rcs0").as_double();
  parameters.snr_param = this->get_parameter("snr").as_double();
  parameters.range_param = this->get_parameter("range").as_double();
  parameters.az_ang0_param = this->get_parameter("az_ang0").as_double();

  raw_left_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarLeft", 1 ,
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback, this,
    std::placeholders::_1));

  raw_right_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
  "unfilteredRadarRight", 1 ,
  std::bind(
    &ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback, this,
    std::placeholders::_1));
  
  left_right_pub_ = this->create_publisher<radar_msgs::msg::RadarPacket>("processed", 20);

}

void ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{

  RCLCPP_INFO(this->get_logger(), "Subscribing: %d\n", msg->event_id);

  // If scan mode is near
  if(parameters.scan_mode == "near" || parameters.scan_mode == "far")
  {
    // Create a temporary empty packet for when we have a full near/far scan filtered packet ready to be published
    radar_msgs::msg::RadarPacket publish_packet;

    /**
    Send message to scan filter and append detections to buffer packet based on timestamp.
    Check if packet is ready to be published (if timestamp differs). Return true if ready to be published, return false otherwise.
    **/
    if(pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet) == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.event_id);
      left_right_pub_->publish(publish_packet);
    }
  }

  // If scan mode is near_far (Double Buffer)
  else
  {
    // Create a temporary empty packet for when we have a full near and far scan filtered packet ready to be published
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    /**
    Send message to near_far_scan_filter and append detections to the buffer packets based on timestamp.
    Check if packet is ready to be published (30 packets). Return true if ready to be published, return false otherwise.
    **/
    if(pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far))
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet_near_far.event_id);
      left_right_pub_->publish(publish_packet_near_far);
    }
  }
}

void ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscribing: %d\n", msg->event_id);

  if(parameters.scan_mode == "near" || parameters.scan_mode == "far" )
  {
    radar_msgs::msg::RadarPacket publish_packet;

    if(pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet) == true)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.event_id);
      left_right_pub_->publish(publish_packet);
    }
  }
  
  else
  {
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    if(pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far) == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet_near_far.event_id);
      left_right_pub_->publish(publish_packet_near_far);
    }
  }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARSPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
