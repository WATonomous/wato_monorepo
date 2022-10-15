#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"

template<typename T,
  typename = typename std::enable_if<rosidl_generator_traits::is_message<T>::value>::type>
class PubSubSpy
{
public:
  PubSubSpy(
    rclcpp::Node::SharedPtr node,
    rclcpp::Executor::SharedPtr executor,
    const std::string & topic_name,
    int qos_history_size,
    int expected_msgs)
  : node_(node), executor_(executor), msg_count_(0), expected_msgs_(expected_msgs)
  {
    sub_ = node->create_subscription<T>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_size)),
      std::bind(&PubSubSpy::msg_callback, this, std::placeholders::_1),
      rclcpp::SubscriptionOptions());
  }

  std::pair<T, bool> expect_msg(int max_retries = 100)
  {
    int retries = 0;
    while (msg_queue_.empty() && msg_count_ < expected_msgs_ && retries < max_retries) {
      executor_->spin_some(std::chrono::milliseconds(10));
      retries++;
    }
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%d %d\n", msg_count_, retries);
    if (msg_count_ > expected_msgs_ || retries >= max_retries) {
      return std::make_pair(T{}, false);
    }

    T msg = *msg_queue_.front();
    msg_queue_.pop();
    return std::make_pair(msg, true);
  }

  bool finished() const
  {
    return msg_count_ == expected_msgs_;
  }

private:
  void msg_callback(const std::shared_ptr<T> msg)
  {
    msg_queue_.push(msg);
    msg_count_++;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::shared_ptr<rclcpp::Subscription<T>> sub_;
  int msg_count_;
  int expected_msgs_;
  std::queue<std::shared_ptr<T>> msg_queue_;
};

#endif  // TEST_UTILS_HPP_
