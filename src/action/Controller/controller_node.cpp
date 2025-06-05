#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "your_pkg/controller_interface.hpp"
#include "your_pkg/controllers/pure_pursuit/pure_pursuit_controller.hpp"  // Choose controller


class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        path_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "", 10, std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));

        state_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "", 10, self.state_callback);
        

        odom_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "", 10, )odom_callback);    
        control_publisher_ = create_publisher<ControlCommand>(
            "", 10);

        timer_ = create_wall_timer(0.75,timer_callback);

        controller_ = std::make_shared<PurePursuitController>();  // Swap here
    }

private:
    std::shared_ptr<ControllerInterface> controller_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr state_sub_;
    rclcpp::Publisher<ControlCommand>::SharedPtr control_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VehicleState current_state;
    std::vector<std::pair<double, double>> path;

    void odom_callback(){
        current_state.velocity = msg->velocity;
    }

    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
       
        for (const auto& pose : msg->poses) {
            path.emplace_back(pose.position.x, pose.position.y);
        }
        controller_->setPath(path);
    }
    void state_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        current_state.x = msg->position.x;
        current_state.y = msg->position.y;
        current_state.theta = tf2::getYaw(msg->orientation);
    }
    void timer_callback() {
        // Timer callbacalso do we know what the route output wil be?k logic if needed
        ControlCommand cmd = controller_->computeControl(state);
        publishCommand(cmd);
    }

    void publishCommand(const ControlCommand& cmd) {
        // Publish to drive topic
        control_publisher_->publish(cmd);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}