#include "odom.hpp"

    WheelOdometry::WheelOdometry() : Node("sensor_subscriber"), x_(0.0), y_(0.0), theta_(0.0), previous_time_(this->now()) {
        leftrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
            "/motor_encoder_left", 10, std::bind(&WheelOdometry::defineVariables, this, std::placeholders::_1, "left_encoder")
        );

        rightrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
            "/motor_encoder_right", 10, std::bind(&WheelOdometry::defineVariables, this, std::placeholders::_1, "right_encoder")
        );

        steering_angle = this->create_subscription<std_msgs::msg::Float64>(
            "/steering_angle", 10, std::bind(&WheelOdometry::defineVariables, this, std::placeholders::_1, "steering_angle")
        );

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/bicycle_model_output", 10);
    }

    void WheelOdometry::defineVariables(const std_msgs::msg::Float64::SharedPtr msg, const std::string &source){
        if (source == "left_encoder") {
            left_wheel_speed = msg->data;
        } else if (source == "right_encoder") {
            right_wheel_speed = msg->data;
        } else if (source == "steering_angle") {
            steering_angle = msg->data;
        }
    }

    void WheelOdometery::bicycleModel() {

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

        RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
        publisher_->publish(odom_message);
    }

 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}