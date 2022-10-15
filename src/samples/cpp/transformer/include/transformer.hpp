#ifndef TRANSFORMER_HPP_
#define TRANSFORMER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "transformer/msg/unfiltered.hpp"
#include "transformer/msg/filtered.hpp"
#include "transformer/msg/filtered_array.hpp"

/**
 * Implementation of a ROS2 node that converts unfiltered messages to filtered_array
 * messages.
 * 
 * Listens to the "unfiltered" topic and filters out data with invalid fields
 * and odd timestamps. Once the node collects BUFFER_CAPACITY messages it packs
 * the processed messages into an array and publishes it to the "filtered" topic.
 * 
 */
class Transformer : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 100 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 100;

  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

public:
  /**
   * Construct a new Transformer object.
   */
  Transformer();

  /**
   * A ROS2 subscription node callback used to process raw data from the
   * "unfiltered" topic and publish to the "filtered" topic.
   *
   * @param msg a raw message from the "unfiltered" topic
   */
  void unfiltered_callback(
    const transformer::msg::Unfiltered::SharedPtr msg);

private:
  /**
   * Convert unfiltered ROS2 message to filtered message.
   *
   * @param[in] unfiltered a raw message
   * @param[out] filtered a processed message
   * @returns whether the raw message is valid
   */
  bool filtered_transformer(
    const transformer::msg::Unfiltered::SharedPtr unfiltered,
    transformer::msg::Filtered & filtered);

  /**
   * Publish array of processed messages to "filtered" topic. Only publishes data
   * if the buffer storing messages reaches its capacity.
   *
   * @param msg a processed message to be published
   */
  void publish_buffer(const transformer::msg::Filtered & msg);

  /**
   * Deserializes the data field of the unfiltered ROS2 message.
   * The data field should be of the form "x:$num1;y:$num2;z:$num3;".
   *
   * @param[in] unfiltered the raw message containing serialized data
   * @param[out] filtered the processed message containing deserialized data
   * @returns whether deserialization was successful
   */
  bool deserialize_position(
    const transformer::msg::Unfiltered::SharedPtr unfiltered,
    transformer::msg::Filtered & filtered);

private:
  // ROS2 subscriber listening to the unfiltered topic.
  rclcpp::Subscription<transformer::msg::Unfiltered>::SharedPtr raw_sub_;

  // ROS2 publisher sending processed messages to the filtered topic.
  rclcpp::Publisher<transformer::msg::FilteredArray>::SharedPtr transform_pub_;

  // Buffer storing processed messages until BUFFER_CAPACITY. Clear after
  // messages are published.
  std::vector<transformer::msg::Filtered> buffer_;
};

#endif  // TRANSFORMER_HPP_
