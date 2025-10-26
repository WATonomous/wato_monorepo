#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_base.h"
#include "bt_closedoor.h"
#include "bt_decorator.h"
#include "bt_enterroom.h"
#include "bt_fallback.h"
#include "bt_isdoorclosed.h"
#include "bt_opendoor.h"
#include "bt_sequence.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<isDoorClosed>("isDoorClosed");
    factory.registerNodeType<OpenDoor>("OpenDoor");
    factory.registerNodeType<Decorator>("Decorator");
    factory.registerNodeType<EnterRoom>("EnterRoom");
    factory.registerNodeType<CloseDoor>("CloseDoor");

    auto tree = factory.createTreeFromText("tree.xml");

    rclcpp::Rate rate(1.0); // sends signal (tick)
    while(rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();

        // Stop if tree finishes
        if (status == BT::NodeStatus::SUCCESS)
        {
            std::cout << "Tree completed successfully!\n";
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            std::cout << "Tree failed.\n";
            break;
        }

        rate.sleep(); // regulates signal timing
    }
}