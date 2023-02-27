#ifndef AGGREGATOR_NODE_HPP_
#define AGGREGATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/filtered_array_average.hpp"

#include "averager.hpp"

/**
 * Implementation of a ROS2 node that listens to the "unfiltered" and "filtered"
 * topics and echoes the operating frequency of the topic to the console.
 */
class AveragerNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Aggregator node constructor.
   */
  AveragerNode();

private:

  /**
   * A ROS2 subscription node callback used to unpack processed data from the
   * "filtered" topic and echo the operating frequency of the topic to the console.
   *
   * @param msg a processed message from the "filtered" topic
   */
  void filtered_callback(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

  // ROS2 publisher sending raw messages to the unfiltered topic.
  rclcpp::Publisher<sample_msgs::msg::FilteredArrayAverage>::SharedPtr data_pub_;

  // ROS2 subscriber listening to the filtered array topic.
  rclcpp::Subscription<sample_msgs::msg::FilteredArray>::SharedPtr raw_sub_;

  // Object containing methods to determine the operating frequency on topics.
  world_modeling::Averager averager_;
};

#endif  // AVERAGER_NODE_HPP_
