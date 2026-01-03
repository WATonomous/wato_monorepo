#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class Fallback : public BT::ControlNode {

    public:
        Fallback(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) : BT::ControlNode(name, config) {
            node_ = node;
            publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/FallbackStatus", 10);
            subscriber_ = node_->create_subscription<std_msgs::msg::Bool>("/Fallback", 10, std::bind(&Fallback::callback, this, std::placeholders::_1));
        }
        
        BT::NodeStatus tick() override {
            for(int i = 0; i < children_nodes_.size(); i++) {
                if(children_nodes_[i]->executeTick() == BT::NodeStatus::SUCCESS) {
                    return BT::NodeStatus::SUCCESS;
                } else if(children_nodes_[i]->executeTick() == BT::NodeStatus::RUNNING) {
                    return BT::NodeStatus::RUNNING;
                }
            }
            return BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts() {
            return { };
        }

    private:
        void callback(const std_msgs::msg::Bool::SharedPtr msg) {
            status = msg->data;
        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
        bool status;
};