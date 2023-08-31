#ifndef TRANSFORMER_CORE_HPP_
#define TRANSFORMER_CORE_HPP_

#include <vector>

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"

namespace samples
{

/**
 * Implementation for the internal logic for the Transformer ROS2
 * node performing data processing and validation.
 */
class TransformerCore
{
public:
  // Size of buffer before processed messages are published.
  static constexpr int BUFFER_CAPACITY = 10;

public:
  /**
   * Transformer constructor.
   */
  TransformerCore();

  /**
   * Retrieve enqueued messages in buffer.
   *
   * @returns enqueued processed messages
   */
  std::vector<sample_msgs::msg::Filtered> buffer_messages() const;

  /**
   * Removes all messages in buffer. Called by the transformer
   * node after messages have been published to aggregator.
   */
  void clear_buffer();

  /**
   * Validates that the 'valid' field of an unfiltered message
   * is set to true.
   *
   * @param unfiltered a raw message
   * @returns whether message's 'valid' field is set
   */
  bool validate_message(
    const sample_msgs::msg::Unfiltered::SharedPtr unfiltered);

  /**
   * Enqueue message into an array of processed messages to "filtered" topic.
   * Ignores messages once the buffer capacity is reached.
   *
   * @param msg a processed message to be published
   * @returns whether buffer is full after adding new message
   */
  bool enqueue_message(const sample_msgs::msg::Filtered & msg);

  /**
   * Deserializes the data field of the unfiltered ROS2 message.
   * The data field should be of the form "x:$num1;y:$num2;z:$num3;".
   *
   * @param[in] unfiltered the raw message containing serialized data
   * @param[out] filtered the processed message containing deserialized data
   * @returns whether deserialization was successful
   */
  bool deserialize_coordinate(
    const sample_msgs::msg::Unfiltered::SharedPtr unfiltered,
    sample_msgs::msg::Filtered & filtered);

private:
  // Buffer storing processed messages until BUFFER_CAPACITY. Clear after
  // messages are published.
  std::vector<sample_msgs::msg::Filtered> buffer_;
};

}  // namespace samples

#endif  // TRANSFORMER_CORE_HPP_
