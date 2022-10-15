#include "aggregator.hpp"

Aggregator::Aggregator()
: Node("aggregator")
{
  raw_sub_ = this->create_subscription<aggregator::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ,
    std::bind(
      &Aggregator::unfiltered_callback, this,
      std::placeholders::_1));
  filtered_sub_ = this->create_subscription<aggregator::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &Aggregator::filtered_callback, this,
      std::placeholders::_1));
}

void Aggregator::unfiltered_callback(
  const aggregator::msg::Unfiltered::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Listening Unfiltered: %s at %d\n", msg->data.c_str(),
    msg->timestamp);
}

void Aggregator::filtered_callback(
  const aggregator::msg::FilteredArray::SharedPtr msg)
{
  for (auto & packet : msg->packets) {
    RCLCPP_INFO(
      this->get_logger(), "Listening Filtered: X: %f Y: %f, Z: %f at %d\n", packet.pos_x,
      packet.pos_y, packet.pos_z, packet.timestamp);
  }
}
