#include <memory>

#include "average_filter_node.hpp"

AverageFilterNode::AverageFilterNode()
: Node("average_filter"), average_filter_(world_modeling::hd_map::AverageFilter())
{
  average_filter_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &AverageFilterNode::average_filter_sub_callback, this,
      std::placeholders::_1));

  average_filter_pub_ =
    this->create_publisher<sample_msgs::msg::FilteredArrayAverage>("filtered_average", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void AverageFilterNode::average_filter_sub_callback(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  auto average = this->average_filter_.getAverage(msg);
  this->average_filter_publish(average);
}

void AverageFilterNode::average_filter_publish(const sample_msgs::msg::FilteredArrayAverage msg)
{
  RCLCPP_INFO(this->get_logger(), "hiiiii! :D %f %f %f \n", msg.avg_x, msg.avg_y, msg.avg_z);
  average_filter_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AverageFilterNode>());
  rclcpp::shutdown();
  return 0;
}
