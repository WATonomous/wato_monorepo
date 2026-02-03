//sub to odom
//sub to trajectory
//sub to tf
//sub to occupancy grid
//pub ackermann control

/*
PathWithSpeed.msg
std_msgs/Header header
nav_msgs/Path path
float64[] speeds
*/

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp> 
#include "mppi_core.hpp"
#include "mppi_control/msg/path_with_speed.hpp"

class MppiNode : public rclcpp::Node {
public:
    MppiNode() : Node("mppi_node") {

        mppi_core_ = std::make_unique<MppiCore>(100, 1.0, 10, 2.5, 0.1, 0.1, 2.0, 0.5, 1.0);

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

        //trajectory subscriber - path_with_speed
        trajectory_sub_ = this->create_subscription<mppi_control::msg::PathWithSpeed>(
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

    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
        //no tf2 dependency
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Odom received");
        mppi_core_->update_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, quaternion_to_yaw(msg->pose.pose.orientation));
        mppi_core_->update_velocity(msg->twist.twist.linear.x);
    }

    void trajectory_callback(const mppi_control::msg::PathWithSpeed::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Trajectory received with %zu points", msg->path.poses.size());
        //turn trajectory into vector of States
        std::vector<State> traj;
        for (size_t i = 0; i < msg->path.poses.size(); i++) {
            State state;
            state.x = msg->path.poses[i].pose.position.x;
            state.y = msg->path.poses[i].pose.position.y;
            state.yaw = quaternion_to_yaw(msg->path.poses[i].pose.orientation);
            state.v = msg->speeds[i];
            traj.push_back(state);
        }

        mppi_core_->update_trajectory(traj);

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

        control_msg.drive.steering_angle = control_output.delta; // Placeholder value
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
    rclcpp::Subscription<mppi_control::msg::PathWithSpeed>::SharedPtr trajectory_sub_;
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