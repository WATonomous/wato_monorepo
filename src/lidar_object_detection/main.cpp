#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_detector.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_DEBUG(rclcpp::get_logger("tmp_logger"), "Starting LidarDetector node");

    auto node = std::make_shared<LidarDetector>();
    rclcpp::spin(node);

    RCLCPP_DEBUG(rclcpp::get_logger("tmp_logger"), "LidarDetector node shutting down.");
    
    rclcpp::shutdown();

    return 0;
}
