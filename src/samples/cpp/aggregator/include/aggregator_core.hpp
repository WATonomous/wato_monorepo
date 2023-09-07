#ifndef AGGREGATOR_CORE_HPP_
#define AGGREGATOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"

namespace samples
{

/**
 * Implementation of the internal logic used by the Aggregator Node to
 * calculate the operating frequency of topics.
 */
class AggregatorCore
{
public:
  /**
   * Aggregator constructor.
   *
   * @param timestamp the Unix timestamp https://en.wikipedia.org/wiki/Unix_time
   */
  explicit AggregatorCore(int64_t timestamp);

  /**
   * Calculates the operating frequency on the "unfiltered" topic. Handles
   * invalid timestamps and division by zero by returning zero.
   *
   * @returns frequency of messages on "unfiltered" topic
   */
  double raw_frequency() const;
  /**
   * Calculates the operating frequency on the "filtered" topic. Handles
   * invalid timestamps and division by zero by returning zero.
   *
   * @returns frequency of messages on "filtered" topic
   */
  double filtered_frequency() const;

  /**
   * Used to keep track of latest timestamp and number of messages received
   * over the "unfiltered" topic. Should be called before raw_frequency().
   *
   * @param msg
   */
  void add_raw_msg(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);
  /**
   * Used to keep track of latest timestamp and number of messages received
   * over the "filtered" topic. Should be called before filtered_frequency().
   *
   * @param msg
   */
  void add_filtered_msg(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

private:
  // Number of message received on "unfiltered" and "filtered" topics.
  int raw_msg_count_;
  int filtered_msg_count_;

  // Unix timestamp used to determine the amount of time that has passed
  // since the beginning of the program.
  int64_t start_;

  // Unix timestamps for last time a message was received on the "unfiltered"
  // and "filtered" topics.
  int64_t latest_raw_time_;
  int64_t latest_filtered_time_;
};

}  // namespace samples

#endif  // AGGREGATOR_CORE_HPP_
