#include <memory>

#include "sample_node.hpp"

SampleNode::SampleNode()
: Node("sample"), sample_(world_modeling::hd_map::Sample())
{
  sample_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ,
    std::bind(
      &SampleNode::sample_sub_callback, this,
      std::placeholders::_1));

  sample_pub_ =
    this->create_publisher<sample_msgs::msg::Unfiltered>("hd_map_sample_topic", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void SampleNode::sample_sub_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  this->sample_publish(msg);
}

void SampleNode::sample_publish(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  auto pub_msg = sample_msgs::msg::Unfiltered(); 
  RCLCPP_INFO(this->get_logger(), "Publishing Sample Message from HD Map...\n");
  sample_pub_->publish(pub_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}
