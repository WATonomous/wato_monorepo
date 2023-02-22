#include <memory>

#include "average_filter_node.hpp"

AverageFilterNode::AverageFilterNode()
: Node("average_filter"), average_filter_(samples::AverageFilter())
{
  filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &AverageFilterNode::filtered_callback, this,
      std::placeholders::_1));
  average_filter_pub_ =
    this->create_publisher<sample_msgs::msg::FilteredArrayAverage>("filtered_average", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void AverageFilterNode::filtered_callback(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  // if (!transformer_.validate_message(msg)) {
  //   return;
  // }

  auto filteredArrayAverage = sample_msgs::msg::FilteredArrayAverage();
  if (!average_filter_.average_coordinate(msg, filteredArrayAverage)) {
    return;
  }

  // filtered.timestamp = msg->timestamp;
  // filtered.metadata.version = this->get_parameter("version").as_int();
  // filtered.metadata.compression_method =
  //   this->get_parameter("compression_method").as_int();

  // if (average_filter_.enqueue_message(filteredArrayAverage)) {
  //   RCLCPP_INFO(this->get_logger(), "Buffer Capacity Reached. PUBLISHING...\n");
  //   // Publish processed data when the buffer reaches its capacity
  //   sample_msgs::msg::FilteredArrayAverage filtered_msgs;
  //   auto buffer = average_filter_.buffer_messages();
  //   average_filter_.clear_buffer();

  //   // Construct FilteredArray object
  //   for (auto & packet : buffer) {
  //     filtered_msgs.packets.push_back(packet);
  //   }
  //   average_filter_pub_->publish(filtered_msgs);
  // }

  // RCLCPP_INFO(this->get_logger(), "Buffer Capacity Reached. PUBLISHING...\n");
  RCLCPP_INFO(
    this->get_logger(), "X:%f, Y:%f, Z:%f\n",
    filteredArrayAverage.avg_x, filteredArrayAverage.avg_y, filteredArrayAverage.avg_z);
  // RCLCPP_INFO(this->get_logger(), "Publishing: %f\n", filteredArrayAverage.array[0].pos_x);
  sample_msgs::msg::FilteredArrayAverage filtered_msgs;

  average_filter_pub_->publish(filteredArrayAverage);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AverageFilterNode>());
  rclcpp::shutdown();
  return 0;
}
