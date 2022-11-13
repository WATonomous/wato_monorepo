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
