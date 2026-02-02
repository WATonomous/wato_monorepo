#include "local_planning/local_planner_node.hpp"

#include <unordered_map>
#include <limits>
#include <chrono>

#include <std_msgs/msg/int64_multi_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <tf2/utils.hpp>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;


LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Local Planner ROS 2 lifecycle node created");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", this->get_current_state().label().c_str());
}

void LocalPlannerNode::timer_callback(){

  FrenetPoint start = {0.0,0.0,0.0,0.0};
  FrenetPoint target1 = {10.0,0.0,1.6,0.0};
  FrenetPoint target2 = {5.0,5.0,1.6,0.5};
  FrenetPoint target3 = {5.0,-5.0,-0.5,-0.2};
  FrenetPoint target4 = {10.0,-7.0,-1.1,-0.1};

  std::vector<FrenetPath> paths;

  FrenetPath path1 {core_.generate_path(start, target1), 0, 0, 0};
  FrenetPath path2 {core_.generate_path(start, target2), 0, 0, 0};
  FrenetPath path3 {core_.generate_path(start, target3), 0, 0, 0};
  FrenetPath path4 {core_.generate_path(start, target4), 0, 0, 0};
  
  paths.push_back(path1);
  paths.push_back(path2);
  paths.push_back(path2);
  paths.push_back(path4);

  if(paths.size() < 1){
    RCLCPP_INFO(this->get_logger(), "Path has less than one point");
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Path has %zu points", paths.size());
  }

  publish_paths_vis(paths);
}

void LocalPlannerNode::publish_paths_vis(std::vector<FrenetPath> paths){
  // Publish as LINE_STRIP marker
  visualization_msgs::msg::MarkerArray marker_array;
  for(int i = 0; i < paths.size(); i++){
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "planned_path";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Blue line, 0.1m thick
    marker.color.r = 0.0; marker.color.g = 0.5; marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;

    // Add path points
    for (const auto& point : paths.at(i).path) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
  }
  path_vis_pub_->publish(marker_array);
}

double LocalPlannerNode::distance_along_first_lanelet(const lanelet_msgs::msg::Lanelet & ll)
{
  double prev_dist = std::numeric_limits<double>::infinity();
  const geometry_msgs::msg::Point * prev_point = nullptr;
  double arc_dist = 0.0;

  if (!car_pose){
    RCLCPP_WARN(this->get_logger(), "Car Pose has not been received yet");
    return -1.0; 
  }   

  geometry_msgs::msg::Point car_pos = car_pose->pose.position;

  for (const auto & pt : ll.centerline) {

    double dist = core_.get_euc_dist(pt.x, pt.y, car_pos.x, car_pos.y);

    if (dist < prev_dist) {
      if (prev_point) {
        arc_dist += core_.get_euc_dist(prev_point->x, prev_point->y, pt.x, pt.y);
      }
      prev_dist = dist;
      prev_point = &pt;  
    } else {
      return arc_dist;
    }
  }

  return arc_dist;
}

void LocalPlannerNode::get_terminal_points(const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg){

  std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> lanelets;
  std::vector<std::vector<int64_t>> id_order(3);

  int64_t curr_id = msg->current_lanelet_id;

  // Generate unordered map of all lanelets in the msg
  for(auto lanelet: msg->lanelets){
    lanelets[lanelet.id] = lanelet;
  }

  // Create order array of ids for each lane
  auto it_curr = lanelets.find(curr_id);
  if (it_curr == lanelets.end()) return;

  const auto & curr_ll = it_curr->second;
  const int64_t first_lanelet_id[3] = {curr_ll.left_lane_id, curr_ll.id, curr_ll.right_lane_id};

  for (int i = 0; i < 3; i++) {
    const int64_t start_id = first_lanelet_id[i];
    if (start_id < 0) continue;

    auto it = lanelets.find(start_id);
    if (it == lanelets.end()) continue;

    id_order[i].push_back(start_id);

    for (const auto succ_id : it->second.successor_ids) {
      if (lanelets.find(succ_id) != lanelets.end()) {
        id_order[i].push_back(succ_id);
      }
    }
  }

  // Search lanes for terminal points at horizons
  if(id_order[1].empty()) return;
  
  auto it_init = lanelets.find(id_order[1][0]);
  if (it_init == lanelets.end()) return;
  
  double init_dist = distance_along_first_lanelet(it_init->second);
  // Clear previous teriminals
  for (auto & horizon : corridor_terminals) {
    for (auto & terminal : horizon) {
      terminal = {FrenetPoint{}, -1};
    }
  }

  for(size_t lane_idx = 0; lane_idx < id_order.size(); lane_idx++){
    const auto & lane = id_order[lane_idx];
    double arc_length = -init_dist;
    int curr_horizon = 0;
    const geometry_msgs::msg::Point * prev_pt = nullptr;

    for(const int64_t ll_id: lane){
      const auto & centerline = lanelets[ll_id].centerline;
      for(size_t pt_idx = 0; pt_idx < centerline.size(); pt_idx++){
        const auto & pt = centerline[pt_idx];
        if(arc_length < lookahead_s_m[curr_horizon]){
          if(prev_pt){
            arc_length += core_.get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
          }
          prev_pt = &pt;
        }
        else{
          // TODO find angle
          double curvature = (pt_idx < lanelets[ll_id].centerline_curvature.size()) 
                    ? lanelets[ll_id].centerline_curvature[pt_idx] 
                    : 0.0;
          FrenetPoint t_pt = {pt.x, pt.y, 0.0, curvature};
          std::pair<FrenetPoint, int64_t> terminal = {t_pt, ll_id};
          corridor_terminals[curr_horizon][lane_idx] = terminal;
          curr_horizon ++;
          if(curr_horizon >= num_horizons) break;
        }
      }
      if(curr_horizon >= num_horizons) break;
    }
  }
  for (auto & horizon : corridor_terminals) {
    for (auto & terminal : horizon) {
      RCLCPP_INFO(this->get_logger(), "Terminal Point: x: %f, y: %f, theta: %f, kappa: %f", terminal.first.x, terminal.first.y, terminal.first.theta, terminal.first.kappa);
    }
  }
  rclcpp::sleep_for(std::chrono::milliseconds(1500));

  // std::vector<FrenetPath> paths;

  // if (!car_frenet_point) {
  //   RCLCPP_WARN(get_logger(), "Car frenet point not available, skipping path generation");
  //   return;  // or publish empty paths
  // } 

  // for (auto & horizon : corridor_terminals) {
  //   for (auto & terminal : horizon) {
  //     if(terminal.second >= 0){
  //       FrenetPath path {core_.generate_path(car_frenet_point.value(), terminal.first), terminal.second, 0, 0};
  //       paths.push_back(path);
  //     }
  //   }
  // }
  // publish_paths_vis(paths);
}


void LocalPlannerNode::update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg){
  geometry_msgs::msg::PoseStamped ps;
  geometry_msgs::msg::Quaternion q;
  geometry_msgs::msg::Point p;
  FrenetPoint fp;
  
  ps.header = msg->header;
  ps.pose   = msg->pose.pose; 
  p = ps.pose.position;
  q = ps.pose.orientation;

  if(!car_frenet_point){
    fp = FrenetPoint{p.x, p.y, tf2::getYaw(q), 0.0};
  }
  else{
    double ds = core_.get_euc_dist(p.x, p.y, car_pose->pose.position.x, car_pose->pose.position.y);
    double kappa = ds > 0.001 ? (tf2::getYaw(q) - car_frenet_point->theta)/ds : 0.0;
    fp = FrenetPoint{p.x, p.y, tf2::getYaw(q), kappa};
  }
  car_pose = ps;
  car_frenet_point = fp;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Local Planning node");
  path_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("planned_path", 10);
  // timer_ = this->create_wall_timer(500ms, std::bind(&LocalPlannerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Local Planning node");
  // route_ahead_sub_ = create_subscription<lanelet_msgs::msg::RouteAhead>(route_ahead_topic, 10, std::bind(&LocalPlannerNode::create_corridor, this, std::placeholders::_1));
  lanelet_ahead_sub_ = create_subscription<lanelet_msgs::msg::LaneletAhead>(lanelet_ahead_topic, 10, std::bind(&LocalPlannerNode::get_terminal_points, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LocalPlannerNode::update_vehicle_odom, this, std::placeholders::_1));
  path_vis_pub_->on_activate();   RCLCPP_INFO(this->get_logger(), "Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Local Planning node");
  // route_ahead_sub_.reset();
  // lanelet_ahead_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Local Planning node");
  // route_ahead_sub_.reset();
  // lanelet_ahead_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Local Planning node");
  // route_ahead_sub_.reset();
  // lanelet_ahead_sub_.reset();
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