#include "local_planning/local_planner_node.hpp"

#include <map>

#include <std_msgs/msg/int64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>


LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Local Planner ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}


void LocalPlannerNode::create_corridor(const lanelet_msgs::msg::RouteAhead::ConstSharedPtr & msg){
  RCLCPP_INFO(this->get_logger(), "Route Ahead Message Received");

  double init_dist = msg->distance_to_first_m;
  double arc_length = -init_dist;
  int curr_horizon = 0;

  std::map<int, lanelet_msgs::msg::Lanelet> lanelets;

  for(auto ll: msg->lanelets){
    lanelets.insert({ll.id, ll});
  }

  std::vector<FrenetPoint> frenet_centreline;

  for(auto id: msg->ids){
    lanelet_msgs::msg::Lanelet lanelet = lanelets.at(id);
    for(auto point: lanelet.centerline){
      arc_length += core_.get_euc_dist(point.x, point.y, car_pose.pose.position.x, car_pose.pose.position.y);
      if(arc_length > lookahead_s_m[curr_horizon] && curr_horizon < num_horizons-1){
        RCLCPP_INFO(this->get_logger(), "Added terminal point {%f, %f} for horizon %f", point.x, point.y, lookahead_s_m[curr_horizon]);
        // TODO compute theta and curvature
        // TODO also can add the other points based on if lane exists and lane width
        FrenetPoint terminal_point {point.x, point.y, 0.0, 0.0};
        corridor_terminals[curr_horizon][1] = terminal_point;
        curr_horizon++;
      }
      else if(curr_horizon > num_horizons-1){
        break;
      }
    }
  }
}

void LocalPlannerNode::update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg){
  car_pose.header = msg->header;
  car_pose.pose = msg->pose.pose;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Local Planning node");
  RCLCPP_INFO(this->get_logger(), "Node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Local Planning node");
  route_ahead_sub_ = create_subscription<lanelet_msgs::msg::RouteAhead>(route_ahead_topic, 10, std::bind(&LocalPlannerNode::create_corridor, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LocalPlannerNode::update_vehicle_odom, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Local Planning node");
  route_ahead_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Local Planning node");
  route_ahead_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Local Planning node");
  route_ahead_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

RCLCPP_COMPONENTS_REGISTER_NODE(LocalPlannerNode)


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto lc_node = std::make_shared<LocalPlannerNode>(rclcpp::NodeOptions());
    lc_node->configure();
    lc_node->activate();
    rclcpp::spin(lc_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}