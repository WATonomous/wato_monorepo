#include <memory>

#include "../include/radar_conti_ars_408_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;

    rclcpp::NodeOptions options;
    auto radar_conti_ars408_node = std::make_shared<watonomous::radar_conti_ars408>(options);
    
    exec.add_node(radar_conti_ars408_node->get_node_base_interface());
    exec.spin();

    rclcpp::shutdown();
}
