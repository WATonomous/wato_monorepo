#ifndef BEHAVIOUR__NODES__REGISTRAR_HPP_
#define BEHAVIOUR__NODES__REGISTRAR_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

/**
 * @class NodeRegistrarBase
 * @brief Base class for registering behaviour tree nodes.
 */
class NodeRegistrarBase
{
public:
    virtual void register_nodes(BT::BehaviorTreeFactory &,
                                const BT::RosNodeParams &) = 0;
    virtual ~NodeRegistrarBase() = default;
};

#endif // BEHAVIOUR__NODES__REGISTRAR_HPP_