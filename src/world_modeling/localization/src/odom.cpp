#include "odom.hpp"

WheelOdometry::WheelOdometry()
    : Node("sensor_subscriber"), x_(0.0), y_(0.0), theta_(0.0), previous_time_(this->now()) {
  leftrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
      "/left_motor_encoder", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { left_wheel_speed = msg->data; });

  rightrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
      "/right_motor_encoder", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { right_wheel_speed = msg->data; });

  steering_angle_subscription = this->create_subscription<std_msgs::msg::Float64>(
      "/steering_angle", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { steering_angle = msg->data; });

  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/bicycle_model_output", 10);

  timer_ = this->create_wall_timer(1000ms, std::bind(&WheelOdometry::bicycleModel, this));
}

void WheelOdometry::bicycleModel() {
  double linear_velocity = (left_wheel_speed + right_wheel_speed) / 2.0;
  double angular_velocity = linear_velocity * tan(steering_angle) / WHEEL_BASE;

  auto current_time = this->now();
  double delta_t = (current_time - previous_time_).seconds();
  previous_time_ = current_time;

  x_ += linear_velocity * cos(theta_) * delta_t;
  y_ += linear_velocity * sin(theta_) * delta_t;
  theta_ += angular_velocity * delta_t;

  auto odom_message = nav_msgs::msg::Odometry();

  odom_message.header.stamp = current_time;
  odom_message.header.frame_id = "odom";
  odom_message.child_frame_id = "base_link";

  odom_message.pose.pose.position.x = x_;
  odom_message.pose.pose.position.y = y_;
  odom_message.pose.pose.position.z = 0.0;

  odom_message.twist.twist.linear.x = linear_velocity;
  odom_message.twist.twist.angular.z = angular_velocity;

  RCLCPP_DEBUG(this->get_logger(), "Publishing: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
  publisher_->publish(odom_message);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
