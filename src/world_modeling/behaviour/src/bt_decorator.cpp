#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorator_node.h>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

class Decorator : public BT::DecoratorNode {

    public:

        Decorator(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) : BT::DecoratorNode(name, config) {
            node_ = node;
            publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/DecoratorStatus", 10);
            subscriber_ = node_->create_subscription<std_msgs::msg::Bool>("/Decorator", 10, std::bind(&Decorator::doorCallback, this, std::placeholders::_1));
        }

        static PortsList providedPorts() {
            return {BT::InputPort<int>("maxattempt")};
        }

        BT::NodeStatus tick() override {
            int maxattempt = getInput<int>("maxattempt").value();
            while(attempts < maxattempt) {
                if(child_node_->executeTick() == BT::NodeStatus::SUCCESS) {
                    return BT::NodeStatus::SUCCESS;
                }
                attempts++;
            }
            attempts = 0;
            return BT::NodeStatus::FAILURE;
        }

    private:
        void doorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            door_status = msg->data;
        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
        int attempts = 0;
        bool door_status;
};