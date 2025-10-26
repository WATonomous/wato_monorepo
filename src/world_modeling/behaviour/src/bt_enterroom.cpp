#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "bt_base.h"

class EnterRoom : public BT::ActionNodeBase {

    public:
        EnterRoom(const std::string& name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config) {}

        BT::NodeStatus tick() override {
            std::cout << "EnterRoom: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
};