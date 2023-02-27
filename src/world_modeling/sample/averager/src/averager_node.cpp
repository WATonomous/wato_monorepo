#include <chrono>
#include <memory>

#include "averager_node.hpp"

AveragerNode::AveragerNode()
: Node("averager"),
  averager_(world_modeling::Averager())
{
  raw_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &AveragerNode::filtered_callback, this,
      std::placeholders::_1));

  data_pub_ =
    this->create_publisher<sample_msgs::msg::FilteredArrayAverage>("filteredaverage", ADVERTISING_FREQ);
}

void AveragerNode::filtered_callback(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  auto average_msg = sample_msgs::msg::FilteredArrayAverage();
  averager_.average_msg(msg, average_msg);

  RCLCPP_INFO(this->get_logger(), "Published FilteredArrayAverage: x_avg: %f, y_avg: %f, z_avg: %f \n", average_msg.x_avg, average_msg.y_avg, average_msg.z_avg);

  data_pub_->publish(average_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AveragerNode>());
  rclcpp::shutdown();
  return 0;
}
