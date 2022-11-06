#include "gtest/gtest.h"

#include "producer.hpp"

TEST(ProducerTest, ValidSerialization)
{
  auto producer = samples::Producer(11, -12, 0);
  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);

  EXPECT_TRUE(msg.valid);
  EXPECT_EQ(msg.data, "x:11;y:-12;z:0;");
}

TEST(ProducerTest, NoCoordinateUpdate)
{
  auto producer = samples::Producer();
  producer.update_coordinates();

  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:0;y:0;z:0;");
}

TEST(ProducerTest, MultipleCoordinateUpdate)
{
  auto producer = samples::Producer();
  auto msg = sample_msgs::msg::Unfiltered();

  producer.set_velocity(1);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:1;y:1;z:1;");

  producer.set_velocity(-4);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:-3;y:-3;z:-3;");
}
