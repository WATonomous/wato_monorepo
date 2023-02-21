#include <memory>

#include "average_filter_node.hpp"

AverageFilterNode::AverageFilterNode()
: Node("average_filter"), average_filter_(world_modelling::AverageFilter())
{
  raw_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "filtered", ADVERTISING_FREQ,
    std::bind(
      &AverageFilterNode::filtered_callback, this,
      std::placeholders::_1));
  average_pub_ =
    this->create_publisher<world_modelling_msgs::msg::FilterArrayAverage>("filter_array_average", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void AverageFilterNode::filtered_callback(const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  auto filter_array_average = world_modelling_msgs::msg::FilterArrayAverage();

  double x_sum{};
  double y_sum{};
  double z_sum{};
  for (auto packet : msg->packets) { 
    x_sum += packet.pos_x;
    y_sum += packet.pos_y;
    z_sum += packet.pos_z;
  }
  long unsigned int num{msg->packets.size()};

  filter_array_average.array = *msg;
  filter_array_average.average_x = x_sum / num;
  filter_array_average.average_y = y_sum / num;
  filter_array_average.average_z = z_sum / num;

  average_pub_->publish(filter_array_average);

  RCLCPP_INFO(this->get_logger(), "average_x: %f\n average_y: %f\n average_z: %f\n", filter_array_average.average_x, filter_array_average.average_y, filter_array_average.average_z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AverageFilterNode>());
  rclcpp::shutdown();
  return 0;
}
