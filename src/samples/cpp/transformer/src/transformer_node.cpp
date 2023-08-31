#include <memory>

#include "transformer_node.hpp"

TransformerNode::TransformerNode()
: Node("transformer"), transformer_(samples::TransformerCore())
{
  raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "/unfiltered_topic", ADVERTISING_FREQ,
    std::bind(
      &TransformerNode::unfiltered_callback, this,
      std::placeholders::_1));
  transform_pub_ =
    this->create_publisher<sample_msgs::msg::FilteredArray>("/filtered_topic", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void TransformerNode::unfiltered_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  if (!transformer_.validate_message(msg)) {
    return;
  }

  auto filtered = sample_msgs::msg::Filtered();
  if (!transformer_.deserialize_coordinate(msg, filtered)) {
    return;
  }

  filtered.timestamp = msg->timestamp;
  filtered.metadata.version = this->get_parameter("version").as_int();
  filtered.metadata.compression_method =
    this->get_parameter("compression_method").as_int();

  if (transformer_.enqueue_message(filtered)) {
    RCLCPP_INFO(this->get_logger(), "Buffer Capacity Reached. PUBLISHING...");
    // Publish processed data when the buffer reaches its capacity
    sample_msgs::msg::FilteredArray filtered_msgs;
    auto buffer = transformer_.buffer_messages();
    transformer_.clear_buffer();

    // Construct FilteredArray object
    for (auto & packet : buffer) {
      filtered_msgs.packets.push_back(packet);
    }
    transform_pub_->publish(filtered_msgs);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformerNode>());
  rclcpp::shutdown();
  return 0;
}
