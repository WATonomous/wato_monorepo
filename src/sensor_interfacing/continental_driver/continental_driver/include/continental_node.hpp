#ifndef CONTINENTAL_DRIVER_NODE_HPP_
#define CONTINENTAL_DRIVER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "driver.hpp"

class ContinentalNode : public rclcpp::Node
{
public:
  ContinentalNode();
  ~ContinentalNode() = default;

private:
  continental_driver::ContinentalDriver driver_;
  rclcpp::Publisher<continental_msgs::msg::ARSPacket>::SharedPtr radar_pub_;
};

#endif  // CONTINENTAL_DRIVER_NODE_HPP_
