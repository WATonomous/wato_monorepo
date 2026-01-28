//sub to odom
//sub to trajectory
//sub to tf
//sub to occupancy grid
//pub ackermann control

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp> 
#include "mppi_core.hpp"


class MppiNode : public rclcpp::Node {
public:
    MppiNode() : Node("mppi_node") {

        mppi_core_ = std::make_unique<MppiCore>(100, 1.0, 10, 2.5, 0.1, 0.1);

        RCLCPP_INFO(this->get_logger(), "MPPI Node has been started.");

        //topic parameter
        this->declare_parameter<std::string>("odom_topic", "/ego/odom");
        this->declare_parameter<std::string>("trajectory_topic", "/trajectory");
        this->declare_parameter<std::string>("tf_topic", "/tf");
        this->declare_parameter<std::string>("occupancy_grid_topic", "/occupancy_grid");
        this->declare_parameter<std::string>("control_topic", "/carla/ackermann_control/command");

        //get parameters
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("trajectory_topic", trajectory_topic_);
        this->get_parameter("tf_topic", tf_topic_);
        this->get_parameter("occupancy_grid_topic", occupancy_grid_topic_);
        this->get_parameter("control_topic", control_topic_);

        //subscriptions

        //odom subscriber - nav_msgs/msg/Odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&MppiNode::odom_callback, this, std::placeholders::_1));

        //trajectory subscriber - trajectory_msgs/msg/JointTrajectory
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            trajectory_topic_, 10,
            std::bind(&MppiNode::trajectory_callback, this, std::placeholders::_1));

        //tf subscriber - tf2_msgs/msg/TFMessage
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic_, 10,
            std::bind(&MppiNode::tf_callback, this, std::placeholders::_1));

        //occupancy grid subscriber - nav_msgs/msg/OccupancyGrid
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            occupancy_grid_topic_, 10,
            std::bind(&MppiNode::occupancy_grid_callback, this, std::placeholders::_1));
        
        //timer for control command publishing
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MppiNode::publish_control_command, this) );
        //control publisher
        control_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(control_topic_, 10);

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Odom received");
    }

    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Trajectory received");
    }

    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "TF received");
    }

    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Occupancy Grid received");
    }

    void publish_control_command() {
        auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();
        Control_Output control_output = mppi_core_->computeControl();

        control_msg.header.stamp = this->now();

        control_msg.drive.steering_angle = control_output.delta_dot; // Placeholder value
        control_msg.drive.acceleration = control_output.a; // Placeholder value
        //control_msg.drive.speed = 0.1; // Placeholder value

        control_pub_->publish(control_msg);

        RCLCPP_INFO(this->get_logger(), "Control command published");
    }


private:
    std::unique_ptr<MppiCore> mppi_core_;
    std::string odom_topic_;
    std::string trajectory_topic_;
    std::string tf_topic_;
    std::string occupancy_grid_topic_;
    std::string control_topic_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    //control publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MppiNode>());
  rclcpp::shutdown();
  return 0;
}