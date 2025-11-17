#include "rclcpp/rclcpp.hpp"
#include <torch/torch.h>
#include <iostream>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


//seperate .hpp file later


class MppiNode : public rclcpp::Node {
public:
  MppiNode() : Node("mppi_node") {
    RCLCPP_INFO(this->get_logger(), "MPPI node starting…");
    /*
    // Simple libtorch test: tensor creation + addition
    torch::Tensor a = torch::rand({2, 3});
    torch::Tensor b = torch::rand({2, 3});
    torch::Tensor c = a + b;

    std::cout << "Tensor a:\n" << a << std::endl;
    std::cout << "Tensor b:\n" << b << std::endl;
    std::cout << "Tensor c = a + b:\n" << c << std::endl;
    
    RCLCPP_INFO(this->get_logger(), "Libtorch test complete."); */
    
    //odom callback
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego/odometry", 10,
      std::bind(&MppiNode::odom_callback, this, std::placeholders::_1));

    //vehicle status callback
    vehicle_status_subscriber_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
      "/carla/ego/vehicle_status", 10,
      std::bind(&MppiNode::vehicle_status_callback, this, std::placeholders::_1));

    //vehicle control publisher
    vehicle_control_publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
      "/carla/ego/vehicle_control_cmd", 10);

    //control timer every 10 ms
    //10ms timer
    // control timer every 10 seconds
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MppiNode::publish_vehicle_control, this));
  
  }


  //odom callback function
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received odometry data: Position(%.2f, %.2f, %.2f)", 
                msg->pose.pose.position.x, 
                msg->pose.pose.position.y, 
                msg->pose.pose.position.z);
  };

  //vehicle status callback function
  void vehicle_status_callback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Velocity: %.2f",//, Acceleration: %.2f, Orientation(quat)",//: (%08.2f, %03.2f, %.2f, %.2f)",
                msg->velocity);
                //msg->acceleration); - geometry msgs accel
                //msg->orientation.x,
                //msg->orientation.y,
                //msg->orientation.z,
                //msg->orientation.w);
  };

  //vehicle command publisher
  void publish_vehicle_control() {
    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();
    control_msg.header.stamp = this->now();
    // Example control values
    double throttle = 0.5;
    double steer = 0.0;
    double brake = 0.0;
    control_msg.throttle = throttle;
    control_msg.steer = steer;
    control_msg.brake = brake;
    control_msg.reverse = false;


    vehicle_control_publisher_->publish(control_msg);
    RCLCPP_INFO(this->get_logger(), "Published vehicle control command: Throttle=%.2f, Steer=%.2f, Brake=%.2f",
                throttle, steer, brake);
  };


  private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr vehicle_status_subscriber_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_control_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MppiNode>());
  rclcpp::shutdown();
  return 0;
}
