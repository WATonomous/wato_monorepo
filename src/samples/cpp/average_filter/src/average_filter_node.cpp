#include <memory>

#include "average_filter_node.hpp"

AverageFilterNode::AverageFilterNode()
: Node("average_filter"), averageFilter_(samples::AverageFilter())
{
  filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &AverageFilterNode::filtered_callback, this,
      std::placeholders::_1));
  average_pub_ =
    this->create_publisher<sample_msgs::msg::FilterArrayAverage>("filter_array_average", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void AverageFilterNode::filtered_callback(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  auto filter_array_average = sample_msgs::msg::FilterArrayAverage();
  filter_array_average.arr = *msg;

  int count = 0;
  int x_sum = 0;
  int y_sum = 0;
  int z_sum = 0;
  for (auto element : filter_array_average.arr.packets) {
    x_sum += element.pos_x;
    y_sum += element.pos_y;
    z_sum += element.pos_z;
    count++;
  }

  filter_array_average.avg_x = float(x_sum) / count;
  filter_array_average.avg_y = float(y_sum) / count;
  filter_array_average.avg_z = float(z_sum) / count;

  average_pub_->publish(filter_array_average);

  RCLCPP_INFO(
    this->get_logger(), "Average x: %f\n",
    filter_array_average.avg_x);
  RCLCPP_INFO(
    this->get_logger(), "Average y: %f\n",
    filter_array_average.avg_x);
  RCLCPP_INFO(
    this->get_logger(), "Average z: %f\n",
    filter_array_average.avg_x);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AverageFilterNode>());
  rclcpp::shutdown();
  return 0;
}
