#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "producer.hpp"

using namespace std::chrono_literals;

Producer::Producer(bool disable_timer)
: Node("producer")
{
  data_pub_ = this->create_publisher<producer::msg::Unfiltered>(
    "unfiltered", ADVERTISING_FREQ);
  // Ignore initializing timer for unit tests
  if (!disable_timer) {
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&Producer::timer_callback, this));
  }

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("x_pos", 0);
  this->declare_parameter("y_pos", 0);
  this->declare_parameter("z_pos", 0);
  this->declare_parameter("valid", true);
}

void Producer::timer_callback()
{
  // Retrieve parameters from params.yaml
  int pos_x = this->get_parameter("x_pos").as_int();
  int pos_y = this->get_parameter("y_pos").as_int();
  int pos_z = this->get_parameter("z_pos").as_int();
  bool valid = this->get_parameter("valid").as_bool();

  auto msg = producer::msg::Unfiltered();
  msg.data = "x:" + std::to_string(pos_x) + ";y:" + std::to_string(pos_y) +
    ";z:" + std::to_string(pos_z) + ";";
  msg.timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
  msg.valid = valid;

  RCLCPP_INFO(this->get_logger(), "Publishing: %s\n", msg.data.c_str());
  data_pub_->publish(msg);
}
