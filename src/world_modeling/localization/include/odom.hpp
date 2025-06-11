#ifndef ODOM_HPP
#define ODOM_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

using namespace std::chrono_literals;

class WheelOdometry : public rclcpp::Node {
 public:
  WheelOdometry();

 private:
  void bicycleModel();
  void vehicleStatusCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightrear_wheel_motor_encoder;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_sub;

  // Carla sim data
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_base_;
  double max_steer_angle_;

  double left_wheel_speed;
  double right_wheel_speed;

  double velocity_;
  double steering_angle_;
  double x_;
  double y_;
  double theta_;

  // rclcpp::Time previous_time_;
};

#endif
