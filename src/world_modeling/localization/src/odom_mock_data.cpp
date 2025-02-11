#include "odom_mock_data.hpp"

OdomMockData::OdomMockData() : Node("odom_mock_data") {
  publisher_left_wheel__ = this->create_publisher<std_msgs::msg::Float64>("left_motor_encoder", 10);
  publisher_right_wheel_ = this->create_publisher<std_msgs::msg::Float64>("right_motor_encoder", 10);
  publisher_steering_angle_ =
      this->create_publisher<std_msgs::msg::Float64>("steering_angle", 10);

  // publisher called every 1 second in meters
  timer_right_ =
      this->create_wall_timer(1000ms, std::bind(&OdomMockData::RandomLeftValues, this));
  timer_left_ =
      this->create_wall_timer(1000ms, std::bind(&OdomMockData::RandomRightValues, this));
  // publisher will fake turn values every 12 seconds
  timer_steering_ =
      this->create_wall_timer(12000ms, std::bind(&OdomMockData::RandomSteeringValues, this));
}

void OdomMockData::RandomLeftValues() {
  auto message = std_msgs::msg::Float64();
  // adding 50km/h speed in m/s to fake encoder values
  left_wheel_encoder = 8.33 + ((double)rand() / RAND_MAX) * (13.89 - 8.33);
  message.data = left_wheel_encoder;
  RCLCPP_DEBUG(this->get_logger(), "Publishing: '%.2f'", message.data);
  this->publisher_left_wheel__->publish(message);
}

void OdomMockData::RandomRightValues() {
  auto message = std_msgs::msg::Float64();
  // adding km/h speed in m/s to fake encoder values
  right_wheel_encoder = 8.33 + ((double)rand() / RAND_MAX) * (13.89 - 8.33);
  message.data = right_wheel_encoder;
  RCLCPP_DEBUG(this->get_logger(), "Publishing: '%.2f'", message.data);
  this->publisher_right_wheel_->publish(message);
}

void OdomMockData::RandomSteeringValues() {
  auto message = std_msgs::msg::Float64();
  // Random turn values in radians
  steering_angle = 0.39 + ((double)rand() / RAND_MAX) * (1.57 - 0.39);
  message.data = steering_angle;
  RCLCPP_DEBUG(this->get_logger(), "Publishing: '%.2f'", message.data);
  this->publisher_steering_angle_->publish(message);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomMockData>());
  rclcpp::shutdown();

  return 0;
}
