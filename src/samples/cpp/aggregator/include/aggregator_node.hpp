#ifndef AGGREGATOR_NODE_HPP_
#define AGGREGATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"

#include "aggregator_core.hpp"

/**
 * Implementation of a ROS2 node that listens to the "unfiltered" and "filtered"
 * topics and echoes the operating frequency of the topic to the console.
 */
class AggregatorNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * Aggregator node constructor.
   */
  AggregatorNode();

private:
  /**
   * A ROS2 subscription node callback used to unpack raw data from the "unfiltered"
   * topic and echo the operating frequency of the topic to the console.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void unfiltered_callback(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);

  /**
   * A ROS2 subscription node callback used to unpack processed data from the
   * "filtered" topic and echo the operating frequency of the topic to the console.
   *
   * @param msg a processed message from the "filtered" topic
   */
  void filtered_callback(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

  // ROS2 subscriber listening to the unfiltered topic.
  rclcpp::Subscription<sample_msgs::msg::Unfiltered>::SharedPtr raw_sub_;

  // ROS2 subscriber listening to the filtered topic.
  rclcpp::Subscription<sample_msgs::msg::FilteredArray>::SharedPtr filtered_sub_;

  // Object containing methods to determine the operating frequency on topics.
  samples::AggregatorCore aggregator_;
};

#endif  // AGGREGATOR_NODE_HPP_
