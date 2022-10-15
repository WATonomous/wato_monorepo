#include <functional>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "producer.hpp"

class ProducerTest : public ::testing::Test
{
public:
  ProducerTest()
  : done(false)
  {
    rclcpp::init(0, nullptr);
  }

  ~ProducerTest()
  {
    rclcpp::shutdown();
  }

protected:
  void SetUp(
    std::function<
      void(const pubsub_sample_cpp::msg::Unfiltered::SharedPtr)
    > callback,
    std::shared_ptr<Producer> producer_node,
    int trigger_count)
  {
    auto test_node = rclcpp::Node::make_shared("test_node");
    auto subscriber = test_node->create_subscription<pubsub_sample_cpp::msg::Unfiltered>(
      "unfiltered", Producer::ADVERTISING_FREQ, callback);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2, false);
    executor->add_node(producer_node);
    executor->add_node(test_node);
    std::thread t([executor] {executor->spin();});

    for (int i = 0; i < trigger_count; i++) {
      producer_node->timer_callback();
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

  bool done;
};

TEST_F(ProducerTest, PublishSingleMessage)
{
  int msgs_received = 0;
  int expected_msgs = 1;
  auto callback =
    [&msgs_received, expected_msgs, this](
    const pubsub_sample_cpp::msg::Unfiltered::SharedPtr msg) -> void {
      msgs_received++;
      if (msgs_received >= expected_msgs) {
        done = true;
      }
    };

  auto producer_node = std::make_shared<Producer>(true);
  SetUp(callback, producer_node, expected_msgs);

  EXPECT_EQ(msgs_received, expected_msgs);
}

TEST_F(ProducerTest, PublishMultipleMessages)
{
  int msgs_received = 0;
  int expected_msgs = 5;
  auto callback =
    [&msgs_received, expected_msgs, this](
    const pubsub_sample_cpp::msg::Unfiltered::SharedPtr msg) -> void {
      msgs_received++;
      if (msgs_received >= expected_msgs) {
        done = true;
      }
    };

  auto producer_node = std::make_shared<Producer>(true);
  SetUp(callback, producer_node, expected_msgs);

  EXPECT_EQ(msgs_received, expected_msgs);
}
