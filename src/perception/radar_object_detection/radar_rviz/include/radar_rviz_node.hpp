#ifndef RADAR_RVIZ_NODE_HPP_
#define RADAR_RVIZ_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "radar_msgs/msg/radar_packet.hpp"
#include "radar_msgs/msg/radar_detection.hpp"

#include "radar_rviz.hpp"

/**
* @brief Implementation of a Radar Rviz node that listens to "processed" topic
*/
class RadarRvizProcessorNode : public rclcpp::Node
{
public:
  RadarRvizProcessorNode();

  /**
  * @brief A ROS2 subscription node callback used to unpack filtered data from the "processed"
  *    topic.
  *
  * @param msg a raw message from the "processed" topic
  */
  void process_radar_data_callback(
    const radar_msgs::msg::RadarPacket::SharedPtr msg);

private:
  // ROS2 Subscriber listening to "processed" topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_sub_;

  // ROS2 Publisher that sends point cloud data to "visualization" topic for Rviz.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;

  // An object containing methods for converting radar packets into point clouds.
  processing::RadarRvizProcessor packet_to_rviz_processor_;
};

#endif  // RADAR_RVIZ_NODE_HPP_
