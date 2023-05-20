#include <memory>

#include "sample_node.hpp"

SampleNode::SampleNode()
: Node("sample"), sample_(world_modeling::hd_map::Sample()), routing_(world_modeling::hd_map::HDMapRouting())
{
  sample_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("hd_map", ADVERTISING_FREQ);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10000),
      std::bind(&SampleNode::sample_publish, this));

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void SampleNode::sample_publish()
{
  lanelet::LineStringLayer& linestrings = this->routing_.map_->lineStringLayer;
  
  auto markerArray = world_modeling::hd_map::lineStringsAsMarkerArray(linestrings);  

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  sample_pub_->publish(markerArray);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}
