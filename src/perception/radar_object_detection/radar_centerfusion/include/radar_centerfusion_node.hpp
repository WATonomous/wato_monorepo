#ifndef RADAR_CENTERFUSION_NODE_HPP_
#define RADAR_CENTERFUSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "radar_centerfusion.hpp"

/**
* @brief Implementation of a Radar Centerfusion node that listens to "processed" topic
*/
class RadarCenterFusionNode : public rclcpp::Node
{
public:
  RadarCenterFusionNode();

  /**
  * @brief A ROS2 subscription node callback used to collect data and run inference from the "radar_data"
  *    topic.
  *
  * @param msg a raw message from the "radar_data" topic
  */
  void radar_data_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

private:
  // ROS2 Subscriber listening to "radar_data" topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_sub_;

  // ROS2 Publisher that sends centerfusion model outputs to "centerfusion_inference" topic for Rviz.
  rclcpp::Publisher<radar_msgs::msg::RadarPacket>::SharedPtr raw_pub_;

  // An object containing methods for running the pretrained model of centerfusion
  inference::RadarCenterFusion radar_inference_;
};

#endif  // RADAR_CENTERFUSION_NODE_HPP_