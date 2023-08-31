#include <chrono>
#include <memory>

#include "aggregator_node.hpp"

AggregatorNode::AggregatorNode()
: Node("aggregator"),
  aggregator_(
    samples::AggregatorCore(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()))
{
  raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "/unfiltered_topic", ADVERTISING_FREQ,
    std::bind(
      &AggregatorNode::unfiltered_callback, this,
      std::placeholders::_1));
  filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "/filtered_topic", ADVERTISING_FREQ,
    std::bind(
      &AggregatorNode::filtered_callback, this,
      std::placeholders::_1));
}

void AggregatorNode::unfiltered_callback(
  const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  aggregator_.add_raw_msg(msg);
  RCLCPP_INFO(
    this->get_logger(), "Raw Frequency(msg/s): %f",
    aggregator_.raw_frequency() * 1000);
}

void AggregatorNode::filtered_callback(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  aggregator_.add_filtered_msg(msg);
  RCLCPP_INFO(
    this->get_logger(), "Filtered Frequency(msg/s): %f",
    aggregator_.filtered_frequency() * 1000);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AggregatorNode>());
  rclcpp::shutdown();
  return 0;
}
