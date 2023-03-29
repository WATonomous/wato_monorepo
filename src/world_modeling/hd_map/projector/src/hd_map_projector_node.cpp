#include <memory>

#include "hd_map_projector_node.hpp"

HDMapProjectorNode::HDMapProjectorNode()
: Node("hd_map_projector_node"), projector_(WGS84ToLocalProjector())
{
  project_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ,
    std::bind(
      &HDMapProjectorNode::project_sub_callback, this,
      std::placeholders::_1));

  project_pub_ =
    this->create_publisher<sample_msgs::msg::Filtered>("filtered", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void HDMapProjectorNode::project_sub_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  // Project the coordinates using the WGS84ToLocalProjector
  auto local_coordinates = projector_.project(msg->latitude, msg->longitude);

  // Create a new Filtered message with the projected coordinates
  auto pub_msg = sample_msgs::msg::Filtered();
  pub_msg.x = local_coordinates.x;
  pub_msg.y = local_coordinates.y;

  // Publish the Filtered message
  RCLCPP_INFO(this->get_logger(), "Publishing projected coordinates...");
  project_pub_->publish(pub_msg);
}

int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<HDMapProjectorNode>());
rclcpp::shutdown();
return 0;
}
