#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "aggregator.hpp"

class AggregatorMock : public Aggregator
{
public:
  explicit AggregatorMock(int expected_count)
  : Aggregator(), done(false), unfiltered_count(0), filtered_count(0),
    expected_count(expected_count) {}

  void unfiltered_callback(
    const pubsub_sample_cpp::msg::Unfiltered::SharedPtr msg) override
  {
    unfiltered_count++;
    if (unfiltered_count >= expected_count) {
      done = true;
    }
  }

  void filtered_callback(
    const pubsub_sample_cpp::msg::FilteredArray::SharedPtr msg) override
  {
    filtered_count++;
    if (filtered_count >= expected_count) {
      done = true;
    }
  }

  bool done;
  int unfiltered_count;
  int filtered_count;
  int expected_count;
};

TEST(AggregatorTest, ReceiveUnfiltered)
{
  rclcpp::init(0, nullptr);
  int expected_msgs_received = 5;
  auto aggregator_node = std::make_shared<AggregatorMock>(expected_msgs_received);
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto publisher =
    test_node->create_publisher<pubsub_sample_cpp::msg::Unfiltered>(
    "unfiltered", Aggregator::ADVERTISING_FREQ);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 2, false);
  executor->add_node(aggregator_node);
  executor->add_node(test_node);
  std::thread t([executor] {executor->spin();});

  std::vector<pubsub_sample_cpp::msg::Unfiltered> unfiltered_msgs;
  for (int i = 0; i < expected_msgs_received; i++) {
    auto msg = pubsub_sample_cpp::msg::Unfiltered();
    msg.data = "x:3;y:1;z:2;";
    msg.valid = false;
    unfiltered_msgs.push_back(msg);
  }

  for (auto & msg : unfiltered_msgs) {
    publisher->publish(msg);
  }

  auto thread_sleep_ms = std::chrono::milliseconds(10);
  int retries = 0;
  int max_retries = 100;
  while (!aggregator_node->done && retries < max_retries) {
    std::this_thread::sleep_for(thread_sleep_ms);
    retries++;
  }

  executor->cancel();
  t.join();

  EXPECT_EQ(aggregator_node->unfiltered_count, expected_msgs_received);
  EXPECT_EQ(aggregator_node->filtered_count, 0);

  rclcpp::shutdown();
}
