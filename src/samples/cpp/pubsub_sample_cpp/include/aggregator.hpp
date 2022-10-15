#ifndef AGGREGATOR_HPP_
#define AGGREGATOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pubsub_sample_cpp/msg/unfiltered.hpp"
#include "pubsub_sample_cpp/msg/filtered_array.hpp"

/**
 * Implementation of a ROS2 node that listens to the "unfiltered" and "filtered"
 * topics and echoes the data to the console.
 * 
 */
class Aggregator : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 100 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 100;

public:
  /**
   * Construct a new Aggregator object.
   */
  Aggregator();

  /**
   * A ROS2 subscription node callback used to unpack raw data from the "unfiltered"
   * topic and echo its contents to the console.
   * 
   * @param msg a raw message from the "unfiltered" topic
   */
  virtual void unfiltered_callback(
    const pubsub_sample_cpp::msg::Unfiltered::SharedPtr msg);

  /**
   * A ROS2 subscription node callback used to unpack processed data from the
   * "filtered" topic and echo its contents to the console.
   * 
   * @param msg a processed message from the "filtered" topic
   */
  virtual void filtered_callback(
    const pubsub_sample_cpp::msg::FilteredArray::SharedPtr msg);

protected:
  // ROS2 subscriber listening to the unfiltered topic.
  rclcpp::Subscription<pubsub_sample_cpp::msg::Unfiltered>::SharedPtr raw_sub_;

  // ROS2 subscriber listening to the filtered topic.
  rclcpp::Subscription<pubsub_sample_cpp::msg::FilteredArray>::SharedPtr filtered_sub_;
};

#endif  // AGGREGATOR_HPP_
