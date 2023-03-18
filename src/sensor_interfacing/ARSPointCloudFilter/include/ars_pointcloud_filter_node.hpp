#ifndef ARS_POINTCLOUD_FILTER_NODE_HPP_
#define ARS_POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "ars_pointcloud_filter.hpp"

/**
* @brief Implementation of a ROS2 Point Cloud Filter node that listens to "unfilteredRadarLeft" 
*        and "unfilteredRadarRight" topics and publishes filtered radar data to "processed" radar topic.
*/
class ARSPointCloudFilterNode : public rclcpp::Node
{
public:

  ARSPointCloudFilterNode();
  filtering::ARSPointCloudFilter::filter_parameters parameters;

private:
  /**
  * A ROS2 subscription node callback used to unpack raw ARS radar data from the "unfilteredRadarRight"
  * topic 
  *
  * @param msg a raw message from the "unfilteredRadarRight" topic
  */
  void unfiltered_ars_radar_right_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  /**
  * A ROS2 subscription node callback used to unpack raw ARS radar data from the "unfilteredRadarLeft"
  * topic 
  *
  * @param msg a raw message from the "unfilteredRadarLeft" topic
  */
  void unfiltered_ars_radar_left_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  // ROS2 Subscriber listening to "unfilteredRadarLeft" topic
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_left_sub_;

  // ROS2 Subscriber listening to "unfilteredRadarRight" topic
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_right_sub_;

  // ROS2 publisher that sends filtered messages from left and right radar to the "processed" topic.
  rclcpp::Publisher<radar_msgs::msg::RadarPacket>::SharedPtr left_right_pub_;

  // An object containing methods for near and far scan filters
  filtering::ARSPointCloudFilter pointcloudfilter_;
};

#endif  // ARS_POINTCLOUD_FILTER_NODE_HPP_
