#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

class IsDoorClosed : public BT::ActionNodeBase {

    public:
        IsDoorClosed(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) : BT::ActionNodeBase(name, config) {
            node_ = node;
            publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/IsDoorClosedStatus", 10);
            subscriber_ = node_->create_subscription<std_msgs::msg::Bool>("/IsDoorClosed", 10, std::bind(&IsDoorClosed::doorCallback, this, std::placeholders::_1));

            is_door_closed = false;
        }
        
        BT::NodeStatus tick() override {

            if(!has_received_first_message) {
                std::cout << "RUNNING IsDoorClosed" << std::endl;
                return BT::NodeStatus::RUNNING;
            }


            if(is_door_closed) {
                std::cout << "Door is Closed" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::RUNNING;
            }
        }

        static BT::PortsList providedPorts() {
            return { };
        }
    
    private:

        void doorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            is_door_closed = msg->data;
            has_received_first_message = true;
        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
        bool is_door_closed;
        bool has_received_first_message;
};