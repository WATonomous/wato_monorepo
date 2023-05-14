#include "rclcpp/rclcpp.hpp"
#include "path_planning_msgs/msg/carla_ego_vehicle_status.hpp"
#include "embedded_msgs/msg/steering_angle_can.hpp"
#include "path_planning_msgs/msg/environment.hpp"
#include "path_planning_msgs/msg/mpc_output.hpp"
#include "path_planning_msgs/msg/ackermann_drive.hpp"
rclcpp::Publisher<embedded_msgs::msg::SteeringAngleCAN>::SharedPtr steeringPub;
rclcpp::Publisher<path_planning_msgs::msg::AckermannDrive>::SharedPtr carlaPub;

// offset for the rosbag
double xOff = 0;
double yOff = 0;


void publish_steering(path_planning_msgs::msg::CarlaEgoVehicleStatus::SharedPtr vehicle) {
    // steer is between -1 and 1, max steer is 70 degrees (1.22173rad), so multiply steer by rad to get approximate steer
    // rwa = (steer.steering_angle * 0.0629 - 0.1363) * 0.0174533;
    auto steer = embedded_msgs::msg::SteeringAngleCAN();
    steer.steering_angle = (vehicle->control.steer * -70 + 0.1363) / 0.0629;
    steeringPub->publish(steer);
}

void env_callback(path_planning_msgs::msg::Environment::SharedPtr env){
    static bool receivedEnv = false;

    if (!receivedEnv){
        receivedEnv = true;
        xOff = env->global_pose.position.x;
        yOff = env->global_pose.position.y;
    }
}

void mpc_to_carla(path_planning_msgs::msg::MPCOutput::SharedPtr mpcOutput) {
    auto carlaControl = path_planning_msgs::msg::AckermannDrive();
    carlaControl.steering_angle = mpcOutput->steer;
    carlaControl.speed = mpcOutput->accel > 0 ? mpcOutput->accel : 0;
    carlaPub->publish(carlaControl);
}

int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create Node
    auto node = std::make_shared<rclcpp::Node>("carla_mpc_bridge");

    // Wait for service
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "set_initial_pose");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    
    //Check if role name specified
    std::string role_name;
    std::string param_name = "role_name";
    do{
        role_name = parameters_client->get_parameters({"role_name"})[0].get_value<std::string>();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } while(role_name == "");

    rclcpp::Subscription<path_planning_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr steeringAngleSub;
    steeringAngleSub = node->create_subscription<path_planning_msgs::msg::CarlaEgoVehicleStatus>(
        "/carla/" + role_name + "/vehicle_status", 2, 
        publish_steering);

    rclcpp::Subscription<path_planning_msgs::msg::MPCOutput>::SharedPtr mpcSub;
    mpcSub = node->create_subscription<path_planning_msgs::msg::MPCOutput>(
        "mpc_output", 2, 
        mpc_to_carla);
    
    rclcpp::Subscription<path_planning_msgs::msg::Environment>::SharedPtr envSub;
    envSub = node->create_subscription<path_planning_msgs::msg::Environment>(
        "/path_planning/environment", 2, 
        env_callback);

    steeringPub = node->create_publisher<embedded_msgs::msg::SteeringAngleCAN>("steering_data", 1);
    carlaPub = node->create_publisher<path_planning_msgs::msg::AckermannDrive>("/carla/" + role_name + "/ackermann_cmd", 1);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
