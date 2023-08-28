#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "aggregator.hpp"

TEST(AggregatorTest, RawDivisionByZero)
{
  samples::Aggregator aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();
  msg->timestamp = 0;

  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.0, aggregator.raw_frequency());
}

TEST(AggregatorTest, FilteredDivisionByZero)
{
  samples::Aggregator aggregator(1);
  auto filtered_packet = std::make_shared<sample_msgs::msg::FilteredArray>();
  std::vector<sample_msgs::msg::Filtered> msgs;
  auto msg = sample_msgs::msg::Filtered();
  msg.timestamp = 1;
  msgs.push_back(msg);
  filtered_packet->packets = msgs;

  aggregator.add_filtered_msg(filtered_packet);
  EXPECT_DOUBLE_EQ(0.0, aggregator.filtered_frequency());
}

TEST(AggregatorTest, RawFrequencyAddSingleMessage)
{
  samples::Aggregator aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();

  msg->timestamp = 2;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.5, aggregator.raw_frequency());
}

TEST(AggregatorTest, RawFrequencyAddMultipleMessages)
{
  samples::Aggregator aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();

  msg->timestamp = 2;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.5, aggregator.raw_frequency());

  msg->timestamp = 1;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(1.0, aggregator.raw_frequency());

  msg->timestamp = 5;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.6, aggregator.raw_frequency());
}

TEST(AggregatorTest, FilteredUnorderedTimestamps)
{
  samples::Aggregator aggregator(0);
  auto filtered_packet = std::make_shared<sample_msgs::msg::FilteredArray>();
  std::vector<sample_msgs::msg::Filtered> msgs;
  auto msg = sample_msgs::msg::Filtered();

  msg.timestamp = 5;
  msgs.push_back(msg);
  msg.timestamp = 2;
  msgs.push_back(msg);
  msg.timestamp = 3;
  msgs.push_back(msg);
  filtered_packet->packets = msgs;

  aggregator.add_filtered_msg(filtered_packet);
  EXPECT_DOUBLE_EQ(0.2, aggregator.filtered_frequency());
}
