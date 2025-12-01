#include "rclcpp/rclcpp.hpp"
#include <torch/torch.h>
#include <iostream>

//#include <mppi_core.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


//seperate .hpp file later
//will take in a trajectory and output control commands - that trajectory inherently handles speed up and slow down

class MppiNode : public rclcpp::Node {
public:
  MppiNode() : Node("mppi_node") {
    RCLCPP_INFO(this->get_logger(), "MPPI node starting…");

    //mppi_core_ = std::make_shared<MppiCore>();


    //self.goal_publisher = self.create_publisher(PoseStamped, "/carla/ego/goal", 10)
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/carla/ego/goal", 10);


    //odom callback
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego/odometry", 10,
      std::bind(&MppiNode::odom_callback, this, std::placeholders::_1));

    //vehicle status callback
    vehicle_status_subscriber_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
      "/carla/ego/vehicle_status", 10,
      std::bind(&MppiNode::vehicle_status_callback, this, std::placeholders::_1));


    costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MppiNode::costmap_callback, this, std::placeholders::_1));



    //vehicle control publisher
    vehicle_control_publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
      "/carla/ego/vehicle_control_cmd", 10);

      //  add in cpp    # Subscribe to waypoints from CARLA
       // self.waypoints_subscription = self.create_subscription(
      //      Path, "/carla/ego/waypoints", self.waypoints_callback, 10    )
    carla_waypoints_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/carla/ego/waypoints", 10,
      std::bind(&MppiNode::waypoints_callback, this, std::placeholders::_1));
    
    //control timer every 10 ms
    //10ms timer
    // control timer every 10 seconds
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MppiNode::publish_vehicle_control, this));
    
    //publish goal
    goal_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      [this]() {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = 50.0;
        goal_msg.pose.position.y = 0.0;
        goal_msg.pose.orientation.w = 1.0;
        goal_publisher_->publish(goal_msg);
      });
  
  }


  //cost map callback function
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received costmap data: Width=%d, Height=%d", 
                msg->info.width, 
                msg->info.height);
  };
  //odom callback function
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ;
    //RCLCPP_INFO(this->get_logger(), "Received odometry data: Position(%.2f, %.2f, %.2f)", 
    //            msg->pose.pose.position.x, 
    //            msg->pose.pose.position.y, 
    //            msg->pose.pose.position.z);
    // Update position in MPPI core using quat to yaw con
    //double yaw = mppi_core_->quat_to_yaw(
    //    msg->pose.pose.orientation.x,
    //    msg->pose.pose.orientation.y,
    //    msg->pose.pose.orientation.z,
    //    msg->pose.pose.orientation.w
    //);
    //mppi_core_->update_position(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    
  };

  //vehicle status callback function
  void vehicle_status_callback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Velocity: %.2f",//, Acceleration: %.2f, Orientation(quat)",//: (%08.2f, %03.2f, %.2f, %.2f)",
    //            msg->velocity);


    ;
      //msg->acceleration); - geometry msgs accel
                //msg->orientation.x,
                //msg->orientation.y,
                //msg->orientation.z,
                //msg->orientation.w);
    //mppi_core_->update_velocity(msg->velocity);
  };

  //waypoints callback function
  void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received waypoints data: Number of waypoints=%d", 
                static_cast<int>(msg->poses.size()));
    //mppi_core_->update_waypoints(msg);
  };

  //vehicle command publisher
  void publish_vehicle_control() {
    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();
    control_msg.header.stamp = this->now();
    // Example control values
    double throttle = 0.0;
    double steer = 0.0;
    double brake = 0.0;
    control_msg.throttle = throttle;
    control_msg.steer = steer;
    control_msg.brake = brake;
    control_msg.reverse = false;


    vehicle_control_publisher_->publish(control_msg);
    //RCLCPP_INFO(this->get_logger(), "Published vehicle control command: Throttle=%.2f, Steer=%.2f, Brake=%.2f",
    //            throttle, steer, brake);
  };


  private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr vehicle_status_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr carla_waypoints_subscription_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_control_publisher_;
  //cost map subscriber

  //std::shared_ptr<MppiCore> mppi_core_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};



int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MppiNode>());
  rclcpp::shutdown();
  return 0;
}
