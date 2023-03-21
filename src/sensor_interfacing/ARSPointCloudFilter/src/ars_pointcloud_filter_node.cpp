#include <chrono>
#include <memory>

#include "ars_pointcloud_filter_node.hpp"
#include "ars_pointcloud_filter.hpp"

ARSPointCloudFilterNode::ARSPointCloudFilterNode(): Node("ars_point_cloud_filter")
{
  /**
  * @note Default values are already declared in yaml file
  */
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

  raw_left_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>("unfilteredRadarLeft",
  1, std::bind(&ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback,
  this, std::placeholders::_1));

  raw_right_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>("unfilteredRadarRight",
  1, std::bind( &ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback,
  this, std::placeholders::_1));

  left_right_pub_ = this->create_publisher<radar_msgs::msg::RadarPacket>("processed", 20);
}

void ARSPointCloudFilterNode::unfiltered_ars_radar_right_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  /**
  * @brief When scan mode = near or far,
  *        send incoming unfiltered msgs to common scan filter and append filtered detections to buffer packet
  *        based on timestamp. Publish packet if ready to be published.
  */
  if(parameters.scan_mode == "near" || parameters.scan_mode == "far")
  {
    radar_msgs::msg::RadarPacket publish_packet;

    if(pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet))
    {
      left_right_pub_->publish(publish_packet);
    }
  }

  /**
  * @brief When scan mode = nearfar,
  *        send incoming unfiltered msgs to NearFar Scan filter and append filtered detections 
  *        to buffer packets based on double buffer algorithm. Publish packet if ready to be published.
  */
  else
  {
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    if(pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far))
    {
      left_right_pub_->publish(publish_packet_near_far);
    }
  }
}

/**
* @note Implementation below is the same as the right callback function
*/
void ARSPointCloudFilterNode::unfiltered_ars_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  if(parameters.scan_mode == "near" || parameters.scan_mode == "far")
  {
    radar_msgs::msg::RadarPacket publish_packet;

    if(pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet))
    {
      left_right_pub_->publish(publish_packet);
    }
  }

  else
  {
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    if(pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far))
    {
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
