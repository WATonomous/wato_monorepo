#ifndef WORLD_MODELING_HD_MAP_PROJECTOR_NODE_HPP_
#define WORLD_MODELING_HD_MAP_PROJECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/unfiltered.hpp"

#include "wgs84_to_local_projector.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages with WGS84
 * coordinates to filtered_array messages with local metric coordinates.
 *
 * Listens to the "unfiltered" topic and projects the WGS84 coordinates to local
 * metric coordinates. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 */
class HDMapProjectorNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * HD Map Projector node constructor.
   */
  HDMapProjectorNode();

private:
  /**
   * A ROS2 subscription node callback used to process raw data from the
   * "unfiltered" topic, project the WGS84 coordinates to local metric coordinates,
   * and publish to the "filtered" topic.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void project_sub_callback(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);

  // ROS2 subscriber listening to the unfiltered topic.
  rclcpp::Subscription<sample_msgs::msg::Unfiltered>::SharedPtr project_sub_;

  // ROS2 publisher sending processed messages with local metric coordinates to the filtered topic.
  rclcpp::Publisher<sample_msgs::msg::Filtered>::SharedPtr project_pub_;

  // Object that handles the projection from WGS84 to local metric coordinates.
  WGS84ToLocalProjector projector_;
};

#endif  // HD_MAP_PROJECTOR_NODE_HPP_
