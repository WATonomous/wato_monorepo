#include <memory>
#include <chrono>

#include "sample_node.hpp"

using namespace std::chrono_literals;

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

  RCLCPP_INFO(this->get_logger(), "Querying service...\n");
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lanelet_service_client");
  rclcpp::Client<world_modeling_msgs::srv::LaneletService>::SharedPtr client =
    node->create_client<world_modeling_msgs::srv::LaneletService>("lanelet_service");

  auto request = std::make_shared<world_modeling_msgs::srv::LaneletService::Request>();
  request->a = 1;
  request->b = 2;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service to start. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service lanelet_service");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNode>());
  rclcpp::shutdown();
  return 0;
}
