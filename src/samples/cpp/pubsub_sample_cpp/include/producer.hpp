#ifndef PRODUCER_HPP_
#define PRODUCER_HPP_

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "pubsub_sample_cpp/msg/unfiltered.hpp"

/**
 * Implementation of a ROS2 node that generates unfiltered ROS2 messages on a
 * time interval.
 * 
 */
class Producer : public rclcpp::Node
{
public:
  // Configure pubsub nodes to keep last 100 messages.
  // https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
  static constexpr int ADVERTISING_FREQ = 100;

public:
  /**
   * Construct a new Producer object.
   * 
   * @param disable_timer disable the timer used to publish messages on an
   *   interval, used for unit tests
   */
  explicit Producer(bool disable_timer = false);

private:
  /**
   * Timer callback used to publish unfiltered ROS2 messages to the "unfiltered"
   * topic.
   */
  void timer_callback();

  /**
   * Declarations enabling unit tests to test logic of private methods.
   */
  friend class ProducerTest;
  FRIEND_TEST(ProducerTest, PublishSingleMessage);
  FRIEND_TEST(ProducerTest, PublishMultipleMessages);

private:
  // Asynchronous ROS2 timer that calls timer_callback every 100ms.
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS2 publisher sending raw messages to the unfiltered topic.
  rclcpp::Publisher<pubsub_sample_cpp::msg::Unfiltered>::SharedPtr data_pub_;
};

#endif  // PRODUCER_HPP_
