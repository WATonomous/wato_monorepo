#ifndef RADAR_VIS_NODE_HPP_
#define RADAR_VIS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "radar_msgs/msg/radar_detection.hpp"
#include "radar_msgs/msg/radar_packet.hpp"

#include "radar_vis.hpp"

/**
 * @brief Implementation of a Radar visualization node that listens to "processed" topic
 */
class RadarVisNode : public rclcpp::Node {
 public:
  RadarVisNode();

  /**
   * @brief A ROS2 subscription node callback used to unpack filtered radar data from the
   * "processed" topic. The callback listens to radar packets on this topic and processes them in
   * order to visualize them in tools like foxglove and RViz.
   *
   * @param msg a raw message from the "processed" topic
   */
  void process_radar_data_callback(const radar_msgs::msg::RadarPacket::SharedPtr msg);

 private:
  // ROS2 Subscriber listening to "processed" topic.
  rclcpp::Subscription<radar_msgs::msg::RadarPacket>::SharedPtr raw_sub_;

  // ROS2 Publisher that sends point cloud data to "visualization" topic for foxglove/Rviz.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;

  // ROS2 Publisher that sends marker arrow data for foxglove/Rviz.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // An object containing methods for converting radar packets into point clouds.
  visualization::RadarVis packet_converter_;
};

#endif  // RADAR_VIS_NODE_HPP_
