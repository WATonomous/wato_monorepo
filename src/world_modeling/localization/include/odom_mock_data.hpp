#ifndef ODOM_MOCK_DATA_HPP
#define ODOM_MOCK_DATA_HPP

#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class OdomMockData : public rclcpp::Node {
 public:
  OdomMockData();

 private:
  void RandomLeftValues();
  void RandomRightValues();
  void RandomSteeringValues();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_wheel__;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_wheel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_steering_angle_;

  rclcpp::TimerBase::SharedPtr timer_left_;
  rclcpp::TimerBase::SharedPtr timer_right_;
  rclcpp::TimerBase::SharedPtr timer_steering_;

  double left_wheel_encoder;
  double right_wheel_encoder;
  double steering_angle;
};

#endif
