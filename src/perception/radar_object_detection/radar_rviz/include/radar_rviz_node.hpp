#ifndef RADAR_RVIZ_NODE_HPP_
#define RADAR_RVIZ_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

// #include "radar_msgs/msg/radar_packet.hpp"
// #include "radar_msgs/msg/radar_detection.hpp"

#include "radar_rviz.hpp"

/**
* @brief Implementation of a Radar Rviz node that listens to "processed" topic
*/
class RadarRvizProcessorNode : public rclcpp::Node
{
public:
  RadarRvizProcessorNode();

private:
  // /**
  // * @brief A ROS2 subscription node callback used to unpack filtered data from the "processed"
  // *    topic.
  // *
  // * @param msg a raw message from the "processed" topic
  // */
  // void process_radar_data_callback(
  //   const radar_msgs::msg::RadarPacket::SharedPtr msg);

  // // ROS2 Subscriber listening to "processed" topic.
  // rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_sub_;

  // An object containing methods for converting radar packets into point clouds.
  processing::RadarRvizProcessor packet_to_rviz_processor_;
};

#endif  // RADAR_RVIZ_NODE_HPP_