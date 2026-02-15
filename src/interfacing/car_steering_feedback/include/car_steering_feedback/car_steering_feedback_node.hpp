#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <roscco_msg/msg/steering_angle.hpp>

namespace car_steering_feedback
{

class CarSteeringFeedbackNode : public rclcpp::Node
{
public:
  explicit CarSteeringFeedbackNode(const rclcpp::NodeOptions & options);
  ~CarSteeringFeedbackNode();

private:
  void read_loop();

  int sock_{-1};
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  rclcpp::Publisher<roscco_msg::msg::SteeringAngle>::SharedPtr pub_;

  std::string can_interface_;
  double steering_conversion_factor_;
};

}  // namespace car_steering_feedback
