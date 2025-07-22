#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

namespace controller_pkg {

class BaseController : public rclcpp::Node {
public:
  explicit BaseController(const std::string &name)
  : Node(name) {}
  virtual ~BaseController() = default;

protected:
  // must override these in derived classes:
  virtual void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) = 0;
  virtual void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)      = 0;
  virtual void controlLoop()                                                = 0;

  // call in your ctor:
  void setupCommonInterfaces(double control_frequency) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego/odometry", 10,
      std::bind(&BaseController::odomCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/extracted_path", 10,
      std::bind(&BaseController::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
      "/carla/ego/vehicle_control_cmd", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
      std::bind(&BaseController::controlLoop, this));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      path_sub_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr                              timer_;
};

}  // namespace controller_pkg
