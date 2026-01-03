#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

class EnterRoom : public BT::ActionNodeBase {

    public:
        EnterRoom(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) : BT::ActionNodeBase(name, config) {
            node_ = node;
            publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/EnterRoomStatus", 10);
            subscriber_ = node_->create_subscription<std_msgs::msg::Bool>("/EnterRoom", 10, std::bind(&EnterRoom::doorCallback, this, std::placeholders::_1));
        }

        BT::NodeStatus tick() override {
            std::cout << "EnterRoom: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts() {
            return { };
        }

    private:
        void doorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            door_status = msg->data;
        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
        bool door_status;
};