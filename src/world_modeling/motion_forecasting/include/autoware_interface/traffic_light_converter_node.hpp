#include "rclcpp/rclcpp.hpp"

class TrafficLightConverterNode : public rclcpp::Node {
public:
    TrafficLightConverterNode();

private:
    rclcpp::Subscription<wato_test_msgs::TrafficLightElement>::SharedPtr subscription_;
    rclcpp::Publisher<autoware_perception_msgs::msg::traffic_light_group_array>::SharedPtr publisher_;
};