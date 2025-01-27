#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/float64.hpp>
#include<std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node
{
    public:
        OdomSubscriber() : Node("sensor_subscriber"), x_(0.0), y_(0.0), theta_(0.0), previous_time_(this->now()) {
            leftrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::float64>(
                "/motor_encoder_left", 10, std::bind(&OdomSubscriber::defineVariables, this, std::placeholders::_1, "left_encoder")
            );

            rightrear_wheel_motor_encoder = this->create_subscription<std_msgs::msg::float64>(
                "/motor_encoder_right", 10, std::bind(&OdomSubscriber::defineVariables, this, std::placeholders::_1, "right_encoder")
            );

            steering_angle = this->create_subscription<std_msgs::msg::float64>(
                "/steering_angle", 10, std::bind(&OdomSubscriber::defineVariables, this, std::placeholders::_1, "steering_angle")
            );

            publisher_ = this->create_publisher<std_msgs::msg::String>("/bicycle_model_output", 10);
        }

        void defineVariables(const std_msg::msg::float64::SharedPtr msg, const std::string &source){
            if (source == "left_encoder") {
                left_wheel_speed = msg->data;
            } else if (source == "right_encoder") {
                right_wheel_speed = msg->data;
            } else if (source == "steering_angle") {
                steering_angle = msg->data;
            }
        }

        void bicycleModel(const std_msgs::msg::String::SharedPtr msg) {

            double linear_velocity = (left_wheel_speed + right_wheel_speed) / 2.0;
            double angular_velocity = linear_velocity * tan(steering_angle) / WHEEL_BASE;

            auto current_time = this->now();
            double delta_t = (current_time - previous_time_).seconds();
            previous_time_ = current_time;

            x_ += linear_velocity * cos(theta_) * delta_t;
            y_ += linear_velocity * sin(theta_) * delta_t;
            theta_ += angular_velocity * delta_t;

            auto message std_msgs::msg::String();
            message.data = "x: " + std::to_string(x_) + ", y: " + std::to_string(y_) + ", theta: " + std::to_string(theta_);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

    private:
        //subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr leftrear_wheel_motor_encoder;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rightrear_wheel_motor_encoder;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr steering_angle;

        //publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


        const double WHEEL_BASE = 2.65;
        //represents the displacement from odom
        double x_ = 0, y_ = 0, theta_ = 0;
        //which is the speed recieved from the topic which will be in out 
        double left_wheel_speed = 0, right_wheel_speed = 0, steering_angle = 0;

        rclcpp::Time previous_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}