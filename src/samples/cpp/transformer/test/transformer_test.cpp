#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "transformer.hpp"

class TransformerFixtureTest : public ::testing::Test
{
public:
  void SetUp(int enqueue_count)
  {
    auto filtered = sample_msgs::msg::Filtered();
    for (int i = 0; i < enqueue_count; i++) {
      transformer.enqueue_message(filtered);
    }
  }

protected:
  samples::Transformer transformer;
};

class TransformerParameterizedTest
  : public ::testing::TestWithParam<std::tuple<const char *, bool>>
{
protected:
  samples::Transformer transformer;
};

TEST(TransformerTest, FilterInvalidField)
{
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->valid = false;

  auto transformer = samples::Transformer();
  bool valid = transformer.validate_message(unfiltered);
  EXPECT_FALSE(valid);
}

TEST_F(TransformerFixtureTest, BufferCapacity)
{
  SetUp(samples::Transformer::BUFFER_CAPACITY - 1);

  // Place last message that fits in buffer
  auto filtered = sample_msgs::msg::Filtered();
  bool full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  int size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);

  // Attempting to enqueue message to full buffer should fail
  full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);
}

TEST_F(TransformerFixtureTest, ClearBuffer)
{
  // Place 3 messages in buffer
  SetUp(3);

  int size = transformer.buffer_messages().size();
  EXPECT_GT(size, 0);

  transformer.clear_buffer();
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 0);
}

TEST_P(TransformerParameterizedTest, SerializationValidation)
{
  auto [serialized_field, valid] = GetParam();
  auto filtered = sample_msgs::msg::Filtered();
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->data = serialized_field;
  EXPECT_EQ(transformer.deserialize_coordinate(unfiltered, filtered), valid);
}

INSTANTIATE_TEST_CASE_P(
  Serialization, TransformerParameterizedTest,
  ::testing::Values(
    std::make_tuple("x:1;y:2;z:3", false),
    std::make_tuple("z:1;y:2;x:3;", false),
    std::make_tuple("x:1,y:2,z:3", false),
    std::make_tuple("x:3;", false),
    std::make_tuple("x:3;y:2;z:3;", true),
    std::make_tuple("x:3;y:22; z:11;", true)));

// class TransformerTest : public ::testing::Test
// {
// public:
//   void SetUp(int expected_msgs)
//   {
//     rclcpp::init(0, nullptr);
//     ASSERT_TRUE(rclcpp::ok());

//     test_node = std::make_shared<Transformer>();
//     helper_node = rclcpp::Node::make_shared("test_helper_node");
//     executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//     executor->add_node(test_node);
//     executor->add_node(helper_node);

//     spy = std::make_shared<PubSubSpy<sample_msgs::msg::FilteredArray>>(
//       helper_node, executor, "filtered", Transformer::ADVERTISING_FREQ, expected_msgs);

//     unfiltered_pub = helper_node->create_publisher<sample_msgs::msg::Unfiltered>(
//       "unfiltered", Transformer::ADVERTISING_FREQ);
//   }

//   void TearDown() override
//   {
//     executor->cancel();
//     rclcpp::shutdown();
//   }

// protected:
//   std::shared_ptr<Transformer> test_node;
//   rclcpp::Node::SharedPtr helper_node;
//   std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
//   std::shared_ptr<PubSubSpy<sample_msgs::msg::FilteredArray>> spy;
//   rclcpp::Publisher<sample_msgs::msg::Unfiltered>::SharedPtr unfiltered_pub;
// };

// class TransformerParamTest : public TransformerTest,
//   public ::testing::WithParamInterface<const char *>
// {};

// TEST_F(TransformerTest, FilterInvalidField)
// {
//   SetUp(0);

//   std::string serialized_data = "x:1;y:2;z:3;";
//   for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
//     auto msg = sample_msgs::msg::Unfiltered();
//     msg.data = serialized_data;
//     msg.valid = false;
//     msg.timestamp = 0;
//     unfiltered_pub->publish(msg);
//     executor->spin_some(std::chrono::milliseconds(10));
//   }

//   EXPECT_TRUE(spy->finished());
// }

// TEST_F(TransformerTest, FilterOddTimestamps)
// {
//   SetUp(0);

//   std::string serialized_data = "x:1;y:2;z:3;";
//   for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
//     auto msg = sample_msgs::msg::Unfiltered();
//     msg.data = serialized_data;
//     msg.valid = true;
//     msg.timestamp = 2 * i + 1;
//     unfiltered_pub->publish(msg);
//     executor->spin_some(std::chrono::milliseconds(10));
//   }

//   EXPECT_TRUE(spy->finished());
// }

// TEST_P(TransformerParamTest, InvalidPositionSerialization)
// {
//   SetUp(0);

//   std::string serialized_data = GetParam();
//   for (int i = 0; i < Transformer::BUFFER_CAPACITY; i++) {
//     auto msg = sample_msgs::msg::Unfiltered();
//     msg.data = serialized_data;
//     msg.valid = true;
//     msg.timestamp = 0;
//     unfiltered_pub->publish(msg);
//     executor->spin_some(std::chrono::milliseconds(10));
//   }

//   EXPECT_TRUE(spy->finished());
// }

// TEST_F(TransformerTest, PublishSingleBuffer)
// {
//   SetUp(1);

//   std::string serialized_data = "x:11;y:22;z:33;";
//   for (int i = 0; i < Transformer::BUFFER_CAPACITY; i++) {
//     auto msg = sample_msgs::msg::Unfiltered();
//     msg.data = serialized_data;
//     msg.valid = true;
//     msg.timestamp = 0;
//     unfiltered_pub->publish(msg);
//     executor->spin_some(std::chrono::milliseconds(10));
//   }

//   auto [filtered_msgs, status] = spy->expect_msg();
//   ASSERT_TRUE(status);
//   for (auto packet : filtered_msgs.packets) {
//     EXPECT_EQ(packet.pos_x, 11);
//     EXPECT_EQ(packet.pos_y, 22);
//     EXPECT_EQ(packet.pos_z, 33);
//   }

//   EXPECT_TRUE(spy->finished());
// }

// TEST_F(TransformerTest, PublishMultipleBuffers)
// {
//   SetUp(2);

//   std::string serialized_data = "x:1;y:2;z:3;";
//   for (int i = 0; i < 2 * Transformer::BUFFER_CAPACITY; i++) {
//     auto msg = sample_msgs::msg::Unfiltered();
//     msg.data = serialized_data;
//     msg.valid = true;
//     msg.timestamp = 0;
//     unfiltered_pub->publish(msg);
//     executor->spin_some(std::chrono::milliseconds(10));

//     if ((i + 1) % Transformer::BUFFER_CAPACITY == 0) {
//       auto [filtered_msgs, status] = spy->expect_msg();
//       ASSERT_TRUE(status);
//       for (auto packet : filtered_msgs.packets) {
//         EXPECT_EQ(packet.pos_x, 1);
//         EXPECT_EQ(packet.pos_y, 2);
//         EXPECT_EQ(packet.pos_z, 3);
//       }
//     }
//   }

//   EXPECT_TRUE(spy->finished());
// }

// INSTANTIATE_TEST_CASE_P(
//   InvalidSerialization, TransformerParamTest,
//   ::testing::Values("x:1;y:2;z:3", "z:1;y:2;x:3;", "x:1,y:2,z:3", "x:3;"));
