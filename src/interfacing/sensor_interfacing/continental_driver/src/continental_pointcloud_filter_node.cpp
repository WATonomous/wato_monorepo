#include <chrono>
#include <memory>

#include "continental_pointcloud_filter_node.hpp"
#include "continental_pointcloud_filter.hpp"

ContinentalPointCloudFilterNode::ContinentalPointCloudFilterNode()
: Node("continental_point_cloud_filter")
{
  /**
  * @note Default values are already declared in yaml file.
  */
  this->declare_parameter("filter_mode", "continental");
  this->declare_parameter("scan_mode", "near");
  this->declare_parameter("vrel_rad", 0);
  this->declare_parameter("el_ang", 0);
  this->declare_parameter("rcs0", 0);
  this->declare_parameter("snr", 0);
  this->declare_parameter("range", 0);
  this->declare_parameter("az_ang0", 0);

  parameters.scan_mode = this->get_parameter("scan_mode").as_string();
  parameters.vrel_rad_param = this->get_parameter("vrel_rad").as_double();
  parameters.el_ang_param = this->get_parameter("el_ang").as_double();
  parameters.rcs0_param = this->get_parameter("rcs0").as_double();
  parameters.snr_param = this->get_parameter("snr").as_double();
  parameters.range_param = this->get_parameter("range").as_double();
  parameters.az_ang0_param = this->get_parameter("az_ang0").as_double();

  raw_left_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
    "unfilteredRadarLeft",
    1, std::bind(
      &ContinentalPointCloudFilterNode::unfiltered_continental_radar_left_callback,
      this, std::placeholders::_1));

  raw_right_sub_ = this->create_subscription<radar_msgs::msg::RadarPacket>(
    "unfilteredRadarRight",
    1, std::bind(
      &ContinentalPointCloudFilterNode::unfiltered_continental_radar_right_callback,
      this, std::placeholders::_1));

  left_right_pub_ = this->create_publisher<radar_msgs::msg::RadarPacket>("processed", 20);
}

void ContinentalPointCloudFilterNode::unfiltered_continental_radar_right_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  /**
  * @brief Depending on the scan mode ("near", "far", "nearfar"),
  *    send incoming unfiltered msgs to common_scan_filter or near_far_scan_filter.
  *    Append filtered detections to the buffer packets.
  *    Publish packet when ready to be published.
  */
  if (parameters.scan_mode == "near" || parameters.scan_mode == "far") {
    radar_msgs::msg::RadarPacket publish_packet;

    if (pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet)) {
      left_right_pub_->publish(publish_packet);
    }
  } else {
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    if (pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far)) {
      left_right_pub_->publish(publish_packet_near_far);
    }
  }
}

/**
* @note Implementation below is the same as the right callback function.
*/
void ContinentalPointCloudFilterNode::unfiltered_continental_radar_left_callback(
  const radar_msgs::msg::RadarPacket::SharedPtr msg)
{
  if (parameters.scan_mode == "near" || parameters.scan_mode == "far") {
    radar_msgs::msg::RadarPacket publish_packet;

    if (pointcloudfilter_.common_scan_filter(msg, parameters, publish_packet)) {
      left_right_pub_->publish(publish_packet);
    }
  } else {
    radar_msgs::msg::RadarPacket publish_packet_near_far;

    if (pointcloudfilter_.near_far_scan_filter(msg, parameters, publish_packet_near_far)) {
      left_right_pub_->publish(publish_packet_near_far);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContinentalPointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
