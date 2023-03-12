#include <chrono>
#include <memory>

#include "simulation_node.hpp"

SimulationNode::SimulationNode() : Node("simulation"){
  // Subscribe to topic (the 10 is an arbitrary refresh rate)
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/carla/ego_vehicle/lidar", 10,
    std::bind(
      &SimulationNode::lidar_callback, this,
      std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Once the carla ros bridge connects to carla you should see lidar data below");
}

void SimulationNode::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // for(const auto &point : msg->data)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Lidar Data: %d", point);
  // }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  //Create Node 
  auto node = std::make_shared<SimulationNode>();

  std::string role_name;

  if (!node->get_parameter("/carla/ego_vehicle/role_name", role_name)){  
      //RCLCPP_ERROR(node->get_logger(), "/carla/ego_vehicle/role_name is not specified");
      //return 1;
  }


  //ros::Subscriber steeringAngleSub = n.subscribe("/carla/" + role_name + "/vehicle_status", 2, publish_steering);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// #include <chrono>
// #include <memory>

// #include "aggregator_node.hpp"

// AggregatorNode::AggregatorNode()
// : Node("aggregator"),
//   aggregator_(
//     samples::Aggregator(
//       std::chrono::duration_cast<std::chrono::milliseconds>(
//         std::chrono::system_clock::now().time_since_epoch()).count()))
// {
//   raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
//     "unfiltered", ADVERTISING_FREQ,
//     std::bind(
//       &AggregatorNode::unfiltered_callback, this,
//       std::placeholders::_1));
//   filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
//     "filtered", ADVERTISING_FREQ,
//     std::bind(
//       &AggregatorNode::filtered_callback, this,
//       std::placeholders::_1));
// }

// void AggregatorNode::unfiltered_callback(
//   const sample_msgs::msg::Unfiltered::SharedPtr msg)
// {
//   aggregator_.add_raw_msg(msg);
//   RCLCPP_INFO(
//     this->get_logger(), "Raw Frequency(msg/s): %f\n",
//     aggregator_.raw_frequency() * 1000);
// }

// void AggregatorNode::filtered_callback(
//   const sample_msgs::msg::FilteredArray::SharedPtr msg)
// {
//   aggregator_.add_filtered_msg(msg);
//   RCLCPP_INFO(
//     this->get_logger(), "Filtered Frequency(msg/s): %f\n",
//     aggregator_.filtered_frequency() * 1000);
// }

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<AggregatorNode>());
//   rclcpp::shutdown();
//   return 0;
// }
