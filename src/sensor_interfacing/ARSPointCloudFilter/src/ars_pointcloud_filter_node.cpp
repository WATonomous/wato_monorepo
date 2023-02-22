#include <chrono>
#include <memory>

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"



ARSPointCloudFilterNode::ARSPointCloudFilterNode(): Node("ars_point_cloud_filter")
{
  //default values already declared in ars_radar_params.yaml
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

  // Send Unfiltered packets along with filter thresholds 
  if(parameters.scan_mode == "near")
  {
    pointcloudfilter_.near_scan_filter(msg, buffer_packet, parameters.vrel_rad_param, parameters.el_ang_param,
                      parameters.rcs0_param, parameters.snr_param, parameters.range_param, parameters.az_ang0_param, publish_packet);

    if(publish_packet.first == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.second.event_id);
      left_right_pub_->publish(publish_packet.second);
    }

  }

  else if (parameters.scan_mode == "far")
  {
    pointcloudfilter_.far_scan_filter(msg, buffer_packet, parameters.vrel_rad_param, parameters.el_ang_param,
                      parameters.rcs0_param, parameters.snr_param, parameters.range_param, parameters.az_ang0_param, publish_packet);

    if(publish_packet.first == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.second.event_id);
      left_right_pub_->publish(publish_packet.second);
    }
  }

}

void ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscribing: %d\n", msg->event_id);

  // Send Unfiltered packets along with set parameter thresholds to the filter
  if(parameters.scan_mode == "near")
  {
    pointcloudfilter_.near_scan_filter(msg, buffer_packet, parameters.vrel_rad_param, parameters.el_ang_param,
                      parameters.rcs0_param, parameters.snr_param, parameters.range_param, parameters.az_ang0_param, publish_packet);

    if(publish_packet.first == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.second.event_id);
      left_right_pub_->publish(publish_packet.second);
    }

  }
  
  else if (parameters.scan_mode == "far")
  {
    pointcloudfilter_.far_scan_filter(msg, buffer_packet, parameters.vrel_rad_param, parameters.el_ang_param,
                      parameters.rcs0_param, parameters.snr_param, parameters.range_param, parameters.az_ang0_param, publish_packet);

    if(publish_packet.first == true)
    {
      // Publish buffer packet
      RCLCPP_INFO(this->get_logger(), "Publishing %d\n", publish_packet.second.event_id);
      left_right_pub_->publish(publish_packet.second);
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
