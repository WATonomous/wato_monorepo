#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_publisher.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_DEBUG(rclcpp::get_logger("tmp_logger"), "Starting LidarPublisher node");

    auto node = std::make_shared<LidarPublisher>();
    rclcpp::spin(node);

    RCLCPP_DEBUG(rclcpp::get_logger("tmp_logger"), "LidarPublisher node shutting down.");
    
    rclcpp::shutdown();

    

    return 0;
}