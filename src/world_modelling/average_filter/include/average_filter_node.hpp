#ifndef AVERAGE_FILTER_NODE_HPP_
#define AVERAGE_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered_array.hpp"
#include "world_modelling_msgs/msg/filter_array_average.hpp"

#include "average_filter.hpp"

/**
 * Implementation of a ROS2 node that converts FilteredArray messages to FilteredArrayAverage messages
 *
 * Listens to the "filtered" topic and calculates the average x, y, z values.
 * Then publishes the result to the "filter_array_average" topic.
 */
class AverageFilterNode : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 20 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * AverageFilter node constructor.
   */
  AverageFilterNode();

private:
  /**
   * A ROS2 subscription node callback used to process messages from the
   * "filtered" topic and publish to the "filtered_average" topic.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void filtered_callback(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

  // ROS2 subscriber listening to the filtered topic.
  rclcpp::Subscription<sample_msgs::msg::FilteredArray>::SharedPtr raw_sub_;

  // ROS2 publisher sending processed messages to the filtered_average topic.
  rclcpp::Publisher<world_modelling_msgs::msg::FilterArrayAverage>::SharedPtr average_pub_;

  // Object that handles data processing and validation.
  world_modelling::AverageFilter average_filter_;
};

#endif  // AVERAGE_FILTER_NODE_HPP_
