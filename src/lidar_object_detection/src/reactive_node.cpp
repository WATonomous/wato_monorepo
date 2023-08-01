// #include <chrono>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "common_msgs/msg/object.hpp"

// using namespace std::chrono_literals;

// class MinimalPublisher : public rclcpp::Node
// {

// private:
//     void timer_callback()
//     {
//         auto message = common_msgs::msg::Object();                   // CHANGE
//         message.id = this->count_++;                                     // CHANGE
//         RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.id); // CHANGE
//         publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<common_msgs::msg::Object>::SharedPtr publisher_; // CHANGE
//     size_t count_;

// public:
//     MinimalPublisher()
//         : Node("minimal_publisher"), count_(0)
//     {
//         publisher_ = this->create_publisher<common_msgs::msg::Object>("topic", 10); // CHANGE
//         timer_ = this->create_wall_timer(
//             500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }