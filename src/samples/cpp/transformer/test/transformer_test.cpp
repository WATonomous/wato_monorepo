#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "test_utils.hpp"
#include "transformer.hpp"

class TransformerTest : public ::testing::Test
{
public:
  void SetUp(int expected_msgs)
  {
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    test_node = std::make_shared<Transformer>();
    helper_node = rclcpp::Node::make_shared("test_helper_node");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(test_node);
    executor->add_node(helper_node);

    spy = std::make_shared<PubSubSpy<transformer::msg::FilteredArray>>(
      helper_node, executor, "filtered", Transformer::ADVERTISING_FREQ, expected_msgs);

    unfiltered_pub = helper_node->create_publisher<transformer::msg::Unfiltered>(
      "unfiltered", Transformer::ADVERTISING_FREQ);
  }

  void TearDown() override
  {
    executor->cancel();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<Transformer> test_node;
  rclcpp::Node::SharedPtr helper_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<PubSubSpy<transformer::msg::FilteredArray>> spy;
  rclcpp::Publisher<transformer::msg::Unfiltered>::SharedPtr unfiltered_pub;
};

class TransformerParamTest : public TransformerTest,
  public ::testing::WithParamInterface<const char *>
{};

TEST_F(TransformerTest, FilterInvalidField)
{
  SetUp(0);

  std::string serialized_data = "x:1;y:2;z:3;";
  for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
    auto msg = transformer::msg::Unfiltered();
    msg.data = serialized_data;
    msg.valid = false;
    msg.timestamp = 0;
    unfiltered_pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(spy->finished());
}

TEST_F(TransformerTest, FilterOddTimestamps)
{
  SetUp(0);

  std::string serialized_data = "x:1;y:2;z:3;";
  for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
    auto msg = transformer::msg::Unfiltered();
    msg.data = serialized_data;
    msg.valid = true;
    msg.timestamp = 2 * i + 1;
    unfiltered_pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(spy->finished());
}

TEST_P(TransformerParamTest, InvalidPositionSerialization)
{
  SetUp(0);

  std::string serialized_data = GetParam();
  for (int i = 0; i < Transformer::BUFFER_CAPACITY; i++) {
    auto msg = transformer::msg::Unfiltered();
    msg.data = serialized_data;
    msg.valid = true;
    msg.timestamp = 0;
    unfiltered_pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(spy->finished());
}

TEST_F(TransformerTest, PublishSingleBuffer)
{
  SetUp(1);

  std::string serialized_data = "x:11;y:22;z:33;";
  for (int i = 0; i < Transformer::BUFFER_CAPACITY; i++) {
    auto msg = transformer::msg::Unfiltered();
    msg.data = serialized_data;
    msg.valid = true;
    msg.timestamp = 0;
    unfiltered_pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));
  }

  auto [filtered_msgs, status] = spy->expect_msg();
  ASSERT_TRUE(status);
  for (auto packet : filtered_msgs.packets) {
    EXPECT_EQ(packet.pos_x, 11);
    EXPECT_EQ(packet.pos_y, 22);
    EXPECT_EQ(packet.pos_z, 33);
  }

  EXPECT_TRUE(spy->finished());
}

TEST_F(TransformerTest, PublishMultipleBuffers)
{
  SetUp(2);

  std::string serialized_data = "x:1;y:2;z:3;";
  for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
    auto msg = transformer::msg::Unfiltered();
    msg.data = serialized_data;
    msg.valid = true;
    msg.timestamp = 0;
    unfiltered_pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));

    if ((i + 1) % Transformer::BUFFER_CAPACITY == 0) {
      auto [filtered_msgs, status] = spy->expect_msg();
      ASSERT_TRUE(status);
      for (auto packet : filtered_msgs.packets) {
        EXPECT_EQ(packet.pos_x, 1);
        EXPECT_EQ(packet.pos_y, 2);
        EXPECT_EQ(packet.pos_z, 3);
      }
    }
  }

  EXPECT_TRUE(spy->finished());
}

INSTANTIATE_TEST_CASE_P(
  InvalidSerialization, TransformerParamTest,
  ::testing::Values("x:1;y:2;z:3", "z:1;y:2;x:3;", "x:1,y:2,z:3", "x:3;"));
