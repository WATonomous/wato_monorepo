#include <chrono>
#include <memory>
#include <vector>

#include "producer_node.hpp"

ProducerNode::ProducerNode(int delay_ms)
: Node("producer"), producer_(samples::ProducerCore())
{
  data_pub_ =
    this->create_publisher<sample_msgs::msg::Unfiltered>("/unfiltered_topic", ADVERTISING_FREQ);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    std::bind(&ProducerNode::timer_callback, this));

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("pos_x", 0.0);
  this->declare_parameter("pos_y", 0.0);
  this->declare_parameter("pos_z", 0.0);
  this->declare_parameter("velocity", 0.0);

  rclcpp::Parameter pos_x = this->get_parameter("pos_x");
  rclcpp::Parameter pos_y = this->get_parameter("pos_y");
  rclcpp::Parameter pos_z = this->get_parameter("pos_z");
  rclcpp::Parameter velocity = this->get_parameter("velocity");

  producer_.update_position(pos_x.as_double(), pos_y.as_double(), pos_z.as_double());
  producer_.update_velocity(velocity.as_double());

  param_cb_ = this->add_on_set_parameters_callback(
    std::bind(&ProducerNode::parameters_callback, this, std::placeholders::_1));
}

void ProducerNode::timer_callback()
{
  producer_.update_coordinates();

  auto msg = sample_msgs::msg::Unfiltered();
  msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  producer_.serialize_coordinates(msg);

  RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
  data_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult ProducerNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "velocity" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      producer_.update_velocity(parameter.as_int());
      RCLCPP_INFO(this->get_logger(), "Velocity successfully set to %d", parameter.as_int());
      result.successful = true;
    }
  }
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProducerNode>(500));
  rclcpp::shutdown();
  return 0;
}
