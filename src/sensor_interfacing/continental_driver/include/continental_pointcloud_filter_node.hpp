#ifndef CONTINENTAL_POINTCLOUD_FILTER_NODE_HPP_
#define CONTINENTAL_POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "continental_pointcloud_filter.hpp"

/**
* @brief Implementation of a ROS2 Point Cloud Filter node that listens to "unfilteredRadarLeft"
*    and "unfilteredRadarRight" topics and publishes filtered radar data to "processed" radar topic.
*/
class ContinentalPointCloudFilterNode : public rclcpp::Node
{
public:
  ContinentalPointCloudFilterNode();

private:
  filtering::ContinentalPointCloudFilter::filter_parameters parameters;

  /**
  * @brief A ROS2 subscription node callback used to unpack raw ARS radar data from the "unfilteredRadarRight"
  *    topic.
  *
  * @param msg a raw message from the "unfilteredRadarRight" topic
  */
  void unfiltered_continental_radar_right_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  /**
  * @brief A ROS2 subscription node callback used to unpack raw ARS radar data from the "unfilteredRadarLeft"
  *    topic.
  *
  * @param msg a raw message from the "unfilteredRadarLeft" topic
  */
  void unfiltered_continental_radar_left_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

  // ROS2 Subscriber listening to "unfilteredRadarLeft" topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_left_sub_;

  // ROS2 Subscriber listening to "unfilteredRadarRight" topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_right_sub_;

  // ROS2 Publisher that sends filtered messages from left and right radar to the "processed" topic.
  rclcpp::Publisher<radar_msgs::msg::RadarPacket>::SharedPtr left_right_pub_;

  // An object containing methods for near and far scan filters.
  filtering::ContinentalPointCloudFilter pointcloudfilter_;
};

#endif  // CONTINENTAL_POINTCLOUD_FILTER_NODE_HPP_
