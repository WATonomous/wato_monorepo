#include <behaviortree_cpp/bt_factory.h>

class baseBT {
    public:
        virtual BT::NodeStatus tick() = 0;
}