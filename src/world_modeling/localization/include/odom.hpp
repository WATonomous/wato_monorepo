#ifndef ODOM_VALUES
#define ODOM_VALUES

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/float64.hpp>
#include<std_msgs/msg/string.hpp>

#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class WheelOdometry : public rclcpp::Node{

public:
  WheelOdometry();

private:
  void bicycleModel();

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  double left_wheel_speed;
  double right_wheel_speed;
  double steering_angle;
  double x_;
  double y_;
  double theta_;
  double wheel_base_;
  rclcpp::Time previous_time_;
};

#endif
