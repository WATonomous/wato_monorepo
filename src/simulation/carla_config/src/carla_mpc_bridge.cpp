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

#include "embedded_msgs/msg/steering_angle_can.hpp"
#include "path_planning_msgs/msg/ackermann_drive.hpp"
#include "path_planning_msgs/msg/carla_ego_vehicle_status.hpp"
#include "path_planning_msgs/msg/environment.hpp"
#include "path_planning_msgs/msg/mpc_output.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class MPC_Bridge_Node : public rclcpp::Node
{
  // offset for the rosbag
  double xOff = 0;
  double yOff = 0;

  rclcpp::Publisher<embedded_msgs::msg::SteeringAngleCAN>::SharedPtr steeringPub;
  rclcpp::Publisher<path_planning_msgs::msg::AckermannDrive>::SharedPtr carlaPub;

public:
  MPC_Bridge_Node()
  : Node("carla_mpc_bridge")
  {
    // Declare Parameters
    this->declare_parameter("mpc_output_topic", "/mpc_output");
    this->declare_parameter("steering_publisher_topic", "/steering_data");

    // Get Parameters
    std::string mpc_output_topic = this->get_parameter("mpc_output_topic").as_string();
    std::string steering_publisher_topic = this->get_parameter("steering_publisher_topic").as_string();

    // Wait for service
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "set_initial_pose");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Check if role name specified
    std::string role_name;
    std::string param_name = "role_name";
    do {
      role_name = parameters_client->get_parameters({"role_name"})[0].get_value<std::string>();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } while (role_name == "");

    // Subscribe to vehicle status
    rclcpp::Subscription<path_planning_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr steeringAngleSub;
    steeringAngleSub = this->create_subscription<path_planning_msgs::msg::CarlaEgoVehicleStatus>(
      "/carla/" + role_name + "/vehicle_status", 2, std::bind(&MPC_Bridge_Node::publish_steering, this, _1));

    // Subscribe to MPC Output
    rclcpp::Subscription<path_planning_msgs::msg::MPCOutput>::SharedPtr mpcSub;
    mpcSub = this->create_subscription<path_planning_msgs::msg::MPCOutput>(
      mpc_output_topic, 2, std::bind(&MPC_Bridge_Node::mpc_to_carla, this, _1));

    rclcpp::Subscription<path_planning_msgs::msg::Environment>::SharedPtr envSub;
    envSub = this->create_subscription<path_planning_msgs::msg::Environment>(
      "/path_planning/environment", 2, std::bind(&MPC_Bridge_Node::env_callback, this, _1));

    this->steeringPub = this->create_publisher<embedded_msgs::msg::SteeringAngleCAN>(steering_publisher_topic, 1);
    this->carlaPub =
      this->create_publisher<path_planning_msgs::msg::AckermannDrive>("/carla/" + role_name + "/ackermann_cmd", 1);
  }

private:
  void publish_steering(path_planning_msgs::msg::CarlaEgoVehicleStatus::SharedPtr vehicle)
  {
    // steer is between -1 and 1, max steer is 70 degrees (1.22173rad), so
    // multiply steer by rad to get approximate steer rwa =
    // (steer.steering_angle * 0.0629 - 0.1363) * 0.0174533;
    auto steer = embedded_msgs::msg::SteeringAngleCAN();
    steer.steering_angle = (vehicle->control.steer * -70 + 0.1363) / 0.0629;
    this->steeringPub->publish(steer);
  }

  void env_callback(path_planning_msgs::msg::Environment::SharedPtr env)
  {
    static bool receivedEnv = false;

    if (!receivedEnv) {
      receivedEnv = true;
      this->xOff = env->global_pose.position.x;
      this->yOff = env->global_pose.position.y;
    }
  }

  void mpc_to_carla(path_planning_msgs::msg::MPCOutput::SharedPtr mpcOutput)
  {
    auto carlaControl = path_planning_msgs::msg::AckermannDrive();
    carlaControl.steering_angle = mpcOutput->steer;
    carlaControl.speed = mpcOutput->accel > 0 ? mpcOutput->accel : 0;
    this->carlaPub->publish(carlaControl);
  }
};

int main(int argc, char ** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create Node
  auto node = std::make_shared<MPC_Bridge_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
