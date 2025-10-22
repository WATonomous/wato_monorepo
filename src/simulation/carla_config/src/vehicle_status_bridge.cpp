// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "interfacing_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// Bridge node to convert Carla vehicle status messages to custom vehicle status messages
class VehicleStatusBridge : public rclcpp::Node
{
public:
  VehicleStatusBridge()
  : Node("vehicle_status_bridge")
  {
    // Declare and get the output topic parameter
    this->declare_parameter("output_vehicle_status_topic", "/vehicle_status");
    std::string output_topic = this->get_parameter("output_vehicle_status_topic").as_string();

    // Set the maximum steering angle (degrees)
    max_steer_angle_ = 70.0;

    // Create publisher for the converted vehicle status messages
    pub_ = this->create_publisher<interfacing_msgs::msg::VehicleStatus>(output_topic, 10);

    // Subscribe to Carla ego vehicle status messages
    sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
      "/carla/ego/vehicle_status",
      10,  // assumes the role name is ego
      std::bind(&VehicleStatusBridge::vehicle_status_callback, this, _1));
  }

private:
  // Publisher for common vehicle status messages
  rclcpp::Publisher<interfacing_msgs::msg::VehicleStatus>::SharedPtr pub_;
  // Subscriber for Carla ego vehicle status messages
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_;
  // Maximum steering angle in degrees
  double max_steer_angle_;

  // Callback to convert and publish vehicle status messages
  void vehicle_status_callback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
  {
    interfacing_msgs::msg::VehicleStatus out_msg;
    out_msg.header = msg->header;  // Copy header
    out_msg.speed = msg->velocity;  // Copy speed
    // Convert normalized steer to angle in radians
    out_msg.steering_angle = msg->control.steer * max_steer_angle_ * M_PI / 180.0;

    pub_->publish(out_msg);  // Publish converted message
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Initialize ROS 2
  auto node = std::make_shared<VehicleStatusBridge>();  // Create node
  rclcpp::spin(node);  // Spin node
  rclcpp::shutdown();  // Shutdown ROS 2
  return 0;
}
