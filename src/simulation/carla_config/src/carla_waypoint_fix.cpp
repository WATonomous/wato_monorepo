/*

Node to add frame IDs to the waypoints being published by carla_waypoint_publisher

*/

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class Waypoint_Modifier_Node : public rclcpp::Node {
 public:
  Waypoint_Modifier_Node() : Node("carla_waypoint_fix") {
    // Declare Parameters
    this->declare_parameter("input_topic", "/carla/ego/waypointsOld");
    this->declare_parameter("output_topic", "/carla/ego/waypoints");

    // Get Parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    // Subscribe to path
    pathSub = this->create_subscription<nav_msgs::msg::Path>(
        input_topic, 1,
        std::bind(&Waypoint_Modifier_Node::publish_path, this, std::placeholders::_1));

    // Create Publisher
    auto qos = rclcpp::QoS(rclcpp::KeepAll())
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    this->pathPub = this->create_publisher<nav_msgs::msg::Path>(output_topic, qos);
  }

 private:
  void publish_path(nav_msgs::msg::Path::SharedPtr pathIn) {
    std::string frame_id = pathIn->header.frame_id;
    path.header = pathIn->header;
    path.poses = pathIn->poses;
    for (auto &pose : path.poses) {
      pose.header.frame_id = frame_id;
    }
    pathPub->publish(path);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;

  nav_msgs::msg::Path path;
};

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create Node
  auto node = std::make_shared<Waypoint_Modifier_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
