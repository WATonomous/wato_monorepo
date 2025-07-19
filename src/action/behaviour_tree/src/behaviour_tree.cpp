#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <world_modeling_msgs/srv/behaviour_tree_info.hpp>
#include <vector>
class BehaviourTree : public rclcpp::Node {
public:
    BehaviourTree() : Node("behaviour_tree") {
        client_ = this->create_client<world_modeling_msgs::srv::BehaviourTreeInfo>("behaviour_tree_info");
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/extracted_path", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&BehaviourTree::sendRequest, this));
    }

private:
    void sendRequest() {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            // RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            return;
        }

        auto request = std::make_shared<world_modeling_msgs::srv::BehaviourTreeInfo::Request>();
        // RCLCPP_INFO(this->get_logger(), "Sending async service request...");
        auto future = client_->async_send_request(request,
            std::bind(&BehaviourTree::handleResponse, this, std::placeholders::_1));
    }

    void handleResponse(rclcpp::Client<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedFuture future) {

        int count = 0;

        try {
            auto response = future.get();

            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->get_clock()->now();

            for (const auto& lanelet : response->route_list.lanelets) {
                // RCLCPP_INFO(this->get_logger(), "Processing lanelet ID: %d", lanelet.id);
                if (!lanelet.centerline.empty()) {
                    for (const auto& pt : lanelet.centerline) {
                        geometry_msgs::msg::PoseStamped pose_stamped;
                        pose_stamped.header = path_msg.header;
                        pose_stamped.pose.position.x = pt.x;
                        pose_stamped.pose.position.y = pt.y;
                        pose_stamped.pose.position.z = pt.z;

                        path_msg.poses.push_back(pose_stamped);

                        count++;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Lanelet centerline is empty for lanelet ID %d", lanelet.id);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Received response with %zu waypoints", count);

            path_pub_->publish(path_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while processing service response: %s", e.what());
        }
    }

    rclcpp::Client<world_modeling_msgs::srv::BehaviourTreeInfo>::SharedPtr client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourTree>());
    rclcpp::shutdown();
    return 0;
}
