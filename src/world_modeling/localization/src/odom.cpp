#include "odom.hpp"


    WheelOdometry::WheelOdometry()
        : Node("odom"), x_(0.0), y_(0.0), theta_(0.0), previous_time_(this->now()) {

        this->declare_parameter<std::string>("left_wheel_topic", std::string("/motor_encoder_left"));
        this->declare_parameter<std::string>("right_wheel_topic", std::string("/motor_encoder_right"));
        this->declare_parameter<std::string>("steering_topic", std::string("/steering_angle"));
        this->declare_parameter<std::string>("odom_output_topic", std::string("/bicycle_model_output"));
        this->declare_parameter<double>("wheel_base", 2.65);
        this->declare_parameter<int>("odom_publish_rate", 10);


        auto left_motor_topic_ = this->get_parameter("left_wheel_topic").as_string();
        auto right_wheel_topic_ = this->get_parameter("right_wheel_topic").as_string();
        auto steering_topic_ = this->get_parameter("steering_topic").as_string();
        auto calculated_odom_ = this->get_parameter("odom_output_topic").as_string();
        auto odom_publish_rate = this->get_parameter("odom_publish_rate").as_int();
        wheel_base_ = this->get_parameter("wheel_base").as_double();

    leftrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
        left_motor_topic_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { left_wheel_speed = msg->data; });

    rightrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::Float64>(
        right_wheel_topic_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { right_wheel_speed = msg->data; });

    steering_angle_sub = this->create_subscription<std_msgs::msg::Float64>(
        steering_topic_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { steering_angle = msg->data; });

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(calculated_odom_, 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(odom_publish_rate), std::bind(&WheelOdometry::bicycleModel, this));
    }

    void WheelOdometry::bicycleModel() {

        double linear_velocity = (left_wheel_speed + right_wheel_speed) / 2.0;
        double angular_velocity = linear_velocity * tan(steering_angle) / wheel_base_;

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
