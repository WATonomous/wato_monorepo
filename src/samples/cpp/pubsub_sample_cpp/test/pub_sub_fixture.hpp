#ifndef PUB_SUB_FIXTURE_HPP_
#define PUB_SUB_FIXTURE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

template<typename PubMessageType, typename SubMessageType>
void multiple_message_pub_sub_fixture(
  const std::string & pub_topic_name,
  const std::string & sub_topic_name,
  rclcpp::Node::SharedPtr real_node,
  std::function<
    void(const typename SubMessageType::SharedPtr)
  > sub_callback,
  const std::vector<PubMessageType> & msgs,
  int advertising_freq,
  bool & done)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto publisher =
    test_node->create_publisher<PubMessageType>(pub_topic_name, advertising_freq);
  auto subscriber =
    test_node->create_subscription<SubMessageType>(
    sub_topic_name, advertising_freq,
    sub_callback);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 2, false);
  executor->add_node(real_node);
  executor->add_node(test_node);
  std::thread t([executor] {executor->spin();});

  for (auto & msg : msgs) {
    publisher->publish(msg);
  }

  auto thread_sleep_ms = std::chrono::milliseconds(10);
  int retries = 0;
  int max_retries = 100;
  while (!done && retries < max_retries) {
    std::this_thread::sleep_for(thread_sleep_ms);
    retries++;
  }

  executor->cancel();
  t.join();
}

#endif  // PUB_SUB_FIXTURE_HPP_
