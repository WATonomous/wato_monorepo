#include <memory>

#include "grid_node.hpp"

GridNode::GridNode()
: Node("grid"), grid_(world_modeling::occupancy::Grid())
{
  grid_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ,
    std::bind(
      &GridNode::grid_sub_callback, this,
      std::placeholders::_1));

  grid_pub_ =
    this->create_publisher<sample_msgs::msg::Unfiltered>("occupancy_grid_topic", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void GridNode::grid_sub_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  this->grid_publish(msg);
}

void GridNode::grid_publish(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  auto pub_msg = sample_msgs::msg::Unfiltered();
    RCLCPP_INFO(this->get_logger(), "Publishing Grid Message from Occupancy...\n");
  grid_pub_->publish(pub_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridNode>());
  rclcpp::shutdown();
  return 0;
}
