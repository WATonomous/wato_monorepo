#include"odom_mock_data.hpp"

    DummyPublisher::DummyPublisher(): Node("odom"){

        this->declare_parameter<int>("wheel_publish_rate", 1000);
        this->declare_parameter<int>("turn_publish_rate", 12000);
        this->declare_parameter<std::string>("left_wheel_topic", std::string("/motor_encoder_left"));
        this->declare_parameter<std::string>("right_wheel_topic", std::string("/motor_encoder_right"));
        this->declare_parameter<std::string>("steering_topic", std::string("/steering_angle"));
        this->declare_parameter<float>("wheel_speed_min", 8.33);
        this->declare_parameter<float>("wheel_speed_max", 13.89);
        this->declare_parameter<float>("turning_angle_min", 0.39);
        this->declare_parameter<float>("turning_angle_max", 1.57);


        auto wheel_publisher_rate_ = this->get_parameter("wheel_publish_rate").as_int();
        auto wheel_turn_rate_ = this->get_parameter("turn_publish_rate").as_int();
        auto left_motor_topic_ = this->get_parameter("left_wheel_topic").as_string();
        auto right_wheel_topic_ = this->get_parameter("right_wheel_topic").as_string();
        auto steering_topic_ = this->get_parameter("steering_topic").as_string();
        min_speed_value_ = this->get_parameter("wheel_speed_min").as_double();
        max_speed_value_ = this->get_parameter("wheel_speed_max").as_double();
        min_turn_value_ = this->get_parameter("turning_angle_min").as_double();
        max_turn_value_ = this->get_parameter("turning_angle_max").as_double();


        publisher_left_wheel__ = this->create_publisher<std_msgs::msg::Float64>(left_motor_topic_, 10);
        publisher_right_wheel_ = this->create_publisher<std_msgs::msg::Float64>(right_wheel_topic_, 10);
        publisher_steering_angle_ = this->create_publisher<std_msgs::msg::Float64>(steering_topic_, 10);

        //publisher called every 1 second in meters
        timer_right_ = this->create_wall_timer(std::chrono::milliseconds(wheel_publisher_rate_), std::bind(&DummyPublisher::RandomLeftValues, this));
        timer_left_ = this->create_wall_timer(std::chrono::milliseconds(wheel_publisher_rate_), std::bind(&DummyPublisher::RandomRightValues, this));
        //publisher will fake turn values every 12 seconds
        timer_steering_ = this->create_wall_timer(std::chrono::milliseconds(wheel_turn_rate_), std::bind(&DummyPublisher::RandomSteeringValues, this));

    }

    void DummyPublisher::RandomLeftValues(){
        auto message = std_msgs::msg::Float64();
        //adding 50km/h speed in m/s to fake encoder values
        left_wheel_encoder = min_speed_value_ + ((double)rand() / RAND_MAX)* (max_speed_value_- min_speed_value_);
        message.data = left_wheel_encoder;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%.2f'", message.data);
        this->publisher_left_wheel__->publish(message);
    }

    void DummyPublisher::RandomRightValues(){
        auto message = std_msgs::msg::Float64();
        //adding km/h speed in m/s to fake encoder values
        right_wheel_encoder = min_speed_value_ + ((double)rand() / RAND_MAX)* (max_speed_value_- min_speed_value_);
        message.data = right_wheel_encoder;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%.2f'", message.data);
        this->publisher_right_wheel_->publish(message);
    }

    void DummyPublisher::RandomSteeringValues(){
        auto message = std_msgs::msg::Float64();
        //Random turn values in radians
        steering_angle = min_turn_value_ + ((double)rand() / RAND_MAX)* (max_turn_value_ - min_turn_value_);
        message.data = steering_angle;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%.2f'", message.data);
        this->publisher_steering_angle_->publish(message);
    }

        

int main (int argc, char * argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyPublisher>());
    rclcpp::shutdown();

    return 0;
}