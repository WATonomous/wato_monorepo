#include "autoware_interface/traffic_light_converter_node.hpp"

#include "wato_test_msgs/TrafficLightElement.h"
#include "autoware_perception_msgs/msg/traffic_light_group_array.hpp"


namespace autoware_interface {
TrafficLightConverterNode::TrafficLightConverterNode() : Node("traffic_light_converter_node") {
    // Subscribe to internal wato message
    subscription_ = this->create_subscription<wato_test_msgs::TrafficLightGroupArray>(
        "traffic_ligh_group_array", 10,
        [this](const wato_test_msgs::TrafficLightGroupArray::SharedPtr traffic_light_group_array) {

            // convert from perception msg to autoware msg
            autoware_perception_msgs::TrafficLightGroupArray autoware_traffic_light_array;

            // set time
            autoware_traffic_light_array.stamp = traffic_light_group_array.stamp;
            
            // add groups to array msg
            autoware_traffic_light_group_array.resize(traffic_light_group_array.size());
            for (int i=0; i<traffic_light_group_array.size(); i++) {
                wato_test_msgs::TrafficLightGroup traffic_light_group = traffic_light_group_array.groups[i];
                autoware_perception_msgs::TrafficLightGroup autoware_traffic_light_group;

                // set traffic light group id
                autoware_traffic_light_group.traffic_light_group_id = traffic_light_group.traffic_light_group_id;
                
                // add traffic light elements to group
                autoware_traffic_light_group.resize(traffic_light_group.size());
                for (int j=0; j<traffic_light_group.size(); j++) {
                    wato_test_msgs::Traffic_LightElement traffic_light_element = traffic_light_group.elements[i];
                    
                    autoware::perception_msgs::TrafficLightElement autoware_traffic_light_element;
                    autoware_traffic_light_element.color = traffic_light_element.color;
                    autoware_traffic_light_element.shape = traffic_light_element.shape;
                    autoware_traffic_light_element.direction = traffic_light_element.direction;
                    autoware_traffic_light_element.confidence = traffic_light_element.confidence;
                    
                    autoware_traffic_light_group[i] = autoware_traffic_light_element;
                }

                autoware_traffic_light_group_array.groups[i] = autoware_traffic_light_group;
            }
            
            // publish the converted message
            publisher_->publish(autoware_traffic_light_group_array);
        });

    // Publisher for autoware_perception_msgs
    publisher_ = this->create_publisher<autoware_perception_msgs::msg::traffic_light_group_array>(
        "autoware_perception_topic", 10);
}
} // namespace


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_interface::TrafficLightConverterNode)