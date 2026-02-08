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
  
  // Subscription topics
  lanelet_ahead_topic = this->declare_parameter("lanelet_ahead_topic", "/world_modeling/lanelet/lanelet_ahead");
  odom_topic = this->declare_parameter("odom_topic", "/ego/odom");
  costmap_topic = this->declare_parameter("costmap_topic", "/world_modeling/costmap");
  bt_topic = this->declare_parameter("bt_topic", "/behaviour/execute_behaviour");

  // Publishing topics
  planned_paths_vis_topic = this->declare_parameter("planned_paths_vis_topic", "planned_paths_markers");
  final_path_vis_topic = this->declare_parameter("final_paths_vis_topic", "final_path_markers");
  final_path_topic = this->declare_parameter("path_topic", "path");

  // Path generation parameters
  //  - Corridor -
  num_horizons = this->declare_parameter("num_horizons", 3);
  lookahead_s_m = this->declare_parameter("lookahead_distances", std::vector<double>{10.0, 15.0, 20.0});
  
  //  - Costmap -
  cm_params.occupancy_weight = this->declare_parameter("cm_occupancy_weight", 20.0);
  cm_params.lateral_movement_weight = this->declare_parameter("cm_lateral_movement_weight",2.0);
  cm_params.physical_limits_weight = this->declare_parameter("cm_physical_limits_weight",4.0);
  cm_params.preferred_lane_cost = this->declare_parameter("cm_preferred_lane_cost",20.0);
  cm_params.unknown_occupancy_cost = this->declare_parameter("cm_unknown_occupancy_cost",50.0);
  cm_params.max_curvature_change = this->declare_parameter("cm_max_curvature_change",0.1);

  //  - Path Generation -
  pg_params.max_iterations = this->declare_parameter("max_iterations", 20);
  pg_params.steps = this->declare_parameter("path_steps", 20);
  pg_params.tolerance = this->declare_parameter("convergence_tolerance", 0.25);
  pg_params.newton_damping = this->declare_parameter("newton_damping", 0.7);
  pg_params.max_step_size = this->declare_parameter("max_step_size", 1.0);
}

void LocalPlannerNode::update_vehicle_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg){
  geometry_msgs::msg::PoseStamped ps;
  geometry_msgs::msg::Quaternion q;
  geometry_msgs::msg::Point p;
  PathPoint fp;
  double yaw;
  
  ps.header = msg->header;
  ps.pose   = msg->pose.pose; 
  p = ps.pose.position;
  q = ps.pose.orientation;
  yaw = core_.normalise_angle(tf2::getYaw(q));

  if(!car_frenet_point){
    fp = PathPoint{p.x, p.y, yaw, 0.0};
  }
  else{
    double ds = core_.get_euc_dist(p.x, p.y, car_pose->pose.position.x, car_pose->pose.position.y);
    double kappa = ds > 0.001 ? (yaw - car_frenet_point->theta)/ds : 0.0;
    fp = PathPoint{p.x, p.y, yaw, kappa};
  }
  car_pose = ps;
  car_frenet_point = fp;
}

void LocalPlannerNode::update_costmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg){
  costmap = *msg;
}

void LocalPlannerNode::set_preferred_lanes(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg){
  preferred_lanelets.clear();
  for(auto id: msg->preferred_lanelet_ids){
    preferred_lanelets[id] = 1;
  }
}

void LocalPlannerNode::lanelet_update_callback(
  const lanelet_msgs::msg::LaneletAhead::ConstSharedPtr & msg)
{
  std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> lanelets;
  std::vector<std::vector<int64_t>> id_order;

  const int64_t curr_id = msg->current_lanelet_id;

  // Build map of all lanelets in msg
  for (const auto & ll : msg->lanelets) {
    lanelets[ll.id] = ll;
  }

  // Clear previous terminals
  corridor_terminals.clear();

  // Get the order of ids for each lane
  id_order = get_id_order(curr_id, lanelets);

  // Search lanes for terminal points at horizons
  for (size_t lane_idx = 0; lane_idx < id_order.size(); lane_idx++){
    const auto & lane = id_order[lane_idx];

    int curr_horizon = 0;
    bool closest_found = false;
    double arc_length = 0.0;
    const geometry_msgs::msg::Point * prev_pt = nullptr;

    for (const int64_t ll_id : lane) {
      const auto & centerline = lanelets.at(ll_id).centerline;
      
      for (size_t pt_idx = 0; pt_idx < centerline.size(); pt_idx++) {
        if (curr_horizon >= num_horizons) break;

        const auto & pt = centerline[pt_idx];

        if(!closest_found){
          if(point_ahead_of_car(pt)){
            closest_found = true;
          }
          else{
            continue;
          }
        }

        // Horizon sampling
        if (arc_length < lookahead_s_m[curr_horizon]) {
          if (prev_pt) {
            arc_length += core_.get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
          }
          prev_pt = &pt;
        }else {
          PathPoint t_pt = create_terminal_point(pt, pt_idx, centerline, prev_pt);
          corridor_terminals.push_back({t_pt, ll_id});
          curr_horizon++;
        }
      }
      if (curr_horizon >= num_horizons) break;
    }
  }
  plan_and_publish_path();
}

void LocalPlannerNode::plan_and_publish_path(){
  std::vector<Path> paths;

  if (!car_frenet_point) {
    RCLCPP_WARN(get_logger(), "Car frenet point not available");
    return;
  }
  
  for (size_t i = 0; i < corridor_terminals.size(); i++) {
    auto & terminal = corridor_terminals[i];
    
    std::vector<PathPoint> ft_path = core_.generate_path(
        car_frenet_point.value(), terminal.first, pg_params);
    
    if(ft_path.empty()){
      RCLCPP_DEBUG(get_logger(), "Path generation failed for terminal %zu", i);
    } else {
      Path path {ft_path, terminal.second, 0, 0};
      paths.push_back(path);
    }
  }
  
  if(paths.empty()){
    RCLCPP_WARN(get_logger(), "No valid paths generated from %zu terminals", 
                corridor_terminals.size());
    return;
  }

  Path lowest_cost = get_lowest_cost_path(paths);
  publish_planned_paths_vis(paths);
  publish_final_path_vis(lowest_cost);
  publish_final_path(lowest_cost);
}



Path LocalPlannerNode::get_lowest_cost_path(const std::vector<Path> & paths){
  Path lowest_cost_path = paths[0]; // Initialize with first valid path
  double prev_cost = path_cost_function(paths[0], preferred_lanelets.count(paths[0].target_lanelet_id) >= 1, cm_params);

  for(size_t i = 1; i < paths.size(); i++){
    bool preferred_lane = preferred_lanelets.count(paths[i].target_lanelet_id) >= 1;
    double path_cost = path_cost_function(paths[i], preferred_lane, cm_params);

    if(path_cost < prev_cost){
      lowest_cost_path = paths[i];
      prev_cost = path_cost;
    }
  }
  return lowest_cost_path;
}

double LocalPlannerNode::path_cost_function(    
  const Path & path,
  bool preferred_lane,
  CostmapParams params
){

  double path_cost = 0.0;
  double prev_kappa = std::numeric_limits<double>::quiet_NaN();

  //Costmap Params
  if(!costmap.has_value()){
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Costmap not available");
    return std::numeric_limits<double>::infinity();
  }

  double cm_res = costmap->info.resolution;
  double cm_ox = costmap->info.origin.position.x;
  double cm_oy = costmap->info.origin.position.y;
  double cm_w  = costmap->info.width;
  double cm_h  = costmap->info.height;
  
  // Preferred Lane Cost
  if(!preferred_lane){
    path_cost += params.preferred_lane_cost;
  }

  for(const auto & pt: path.path){

    // Occupancy Cost
    const int mx = static_cast<int>(std::floor((pt.x - cm_ox) / cm_res));
    const int my = static_cast<int>(std::floor((pt.y - cm_oy) / cm_res));
    
    if( mx >= 0 && my >= 0 && mx < cm_w && my < cm_h){
      const int idx = my * cm_w + mx;
      const int8_t occ = costmap->data[idx];
      
      if(occ >= 0){
        path_cost += occ*params.occupancy_weight;
      }
      else{
        path_cost += params.unknown_occupancy_cost*params.occupancy_weight;
      }
    }

    // Curvature Change Cost (physical limits)
    if(!std::isnan(prev_kappa)){
      double curvature_change = fabs(pt.kappa - prev_kappa);
      if(curvature_change > params.max_curvature_change){
        path_cost += params.physical_limits_weight * (curvature_change - params.max_curvature_change);
      }
    }

    // Lateral Movement Cost (using curvature for rough approx)
    path_cost += params.lateral_movement_weight * fabs(pt.kappa);  
    
    prev_kappa = pt.kappa;
  }

  return path_cost;
}

std::vector<std::vector<int64_t>> LocalPlannerNode::get_id_order(int64_t curr_id, const std::unordered_map<int64_t, lanelet_msgs::msg::Lanelet> & ll_map){
  std::vector<std::vector<int64_t>> id_order;
  
  auto it_curr = ll_map.find(curr_id);
  if (it_curr == ll_map.end()) return id_order;
  
  const auto & curr_ll = it_curr->second;
  const int64_t first_lanelet_id[3] = {curr_ll.left_lane_id, curr_ll.id, curr_ll.right_lane_id};

  // Initialize starting lanes
  for (auto id : first_lanelet_id) {
    if (id >= 0) {
      id_order.push_back({id});
    }
  }

  // Use index-based loop to allow safe addition during iteration
  for (size_t lane_idx = 0; lane_idx < id_order.size(); ++lane_idx) {
  
    while (true) {
      // DON'T hold a reference across the whole loop
      // auto & lane = id_order[lane_idx];  ← REMOVE THIS
      
      // Check empty at the start of each iteration
      if (id_order[lane_idx].empty()) {
        RCLCPP_ERROR(get_logger(), "Empty lane at index %zu", lane_idx);
        break;
      }

      int64_t current_id = id_order[lane_idx].back();
      
      auto it = ll_map.find(current_id);
      if (it == ll_map.end()) break;
      
      const auto & succs = it->second.successor_ids;
      if (succs.empty()) break;
      
      int64_t succ_id = succs.front();
      if (succ_id < 0 || succ_id == current_id || ll_map.count(succ_id) < 1) break;
      
      // Add first successor - access directly, don't hold reference
      id_order[lane_idx].push_back(succ_id);
      
      // Handle splits for ego lane only
      if (succs.size() > 1 && lane_idx == 1) {
        for (size_t i = 1; i < succs.size(); ++i) {
          if (succs[i] >= 0 && succs[i] != current_id && ll_map.count(succs[i]) > 0) {
            // Copy the lane BEFORE adding to id_order
            std::vector<int64_t> split_lane = id_order[lane_idx];
            
            // Safety check (though shouldn't be needed now)
            if (!split_lane.empty()) {
              split_lane.back() = succs[i];
              id_order.push_back(split_lane);  // Can cause reallocation
            }
          }
        }
      }
    } 
  }
return id_order;
}

bool LocalPlannerNode::point_ahead_of_car(const geometry_msgs::msg::Point & pt) {
  const auto & car_pos = car_pose->pose.position;
  const auto & car_quat = car_pose->pose.orientation;
  
  double car_yaw = core_.normalise_angle(tf2::getYaw(car_quat));
  double car_forward_x = std::cos(car_yaw);
  double car_forward_y = std::sin(car_yaw);
  
  double to_point_x = pt.x - car_pos.x;
  double to_point_y = pt.y - car_pos.y;
  
  double dot = car_forward_x * to_point_x + car_forward_y * to_point_y;
  
  return dot > 0.0;  
}

PathPoint LocalPlannerNode::create_terminal_point(
  const geometry_msgs::msg::Point& pt,
  size_t pt_idx,
  const std::vector<geometry_msgs::msg::Point>& centerline,
  const geometry_msgs::msg::Point* prev_pt)
{
  // Calculate heading
  double angle = 0.0;
  if ((pt_idx + 1) < centerline.size()) {
    // Use forward difference
    const auto& next_pt = centerline[pt_idx + 1];
    angle = core_.get_angle_from_pts(pt.x, pt.y, next_pt.x, next_pt.y);
  } else if (prev_pt) {
    // Use backward difference
    angle = core_.get_angle_from_pts(prev_pt->x, prev_pt->y, pt.x, pt.y);
  }
  
  // Calculate curvature
  double curvature = 0.0;
  if (prev_pt) {
    double prev_angle = core_.get_angle_from_pts(prev_pt->x, prev_pt->y, pt.x, pt.y);
    double ds = core_.get_euc_dist(prev_pt->x, prev_pt->y, pt.x, pt.y);
    
    if (ds > 0.001) {
      double dtheta = core_.normalise_angle(angle - prev_angle);
      curvature = dtheta / ds;
    }
  }
  
  return PathPoint{pt.x, pt.y, angle, curvature};
}

void LocalPlannerNode::publish_planned_paths_vis(const std::vector<Path> & paths){
 
  // Publish paths as LINE_STRIP marker
  visualization_msgs::msg::MarkerArray marker_array;
  for(size_t i = 0; i < paths.size(); i++){
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "planned_path";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

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

  // push back terminal points to compare

  int id = marker_array.markers.size();
  for (auto & terminal : corridor_terminals) {
    if(terminal.second >= 0){
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "terminal_points";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Blue line, 0.1m thick
      marker.color.r = 0.0; marker.color.g = 0.8; marker.color.b = 0.5;
      marker.color.a = 1.0;
      marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5;

      geometry_msgs::msg::Point p;
      p.x = terminal.first.x;
      p.y = terminal.first.y;
      p.z = 0.0;

      marker.pose.position = p;

      marker_array.markers.push_back(marker);
      id++;
    }
  }
  planned_path_vis_pub_->publish(marker_array);
}

void LocalPlannerNode::publish_final_path_vis(const Path & path){
  // Publish as LINE_STRIP marker

  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "final_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Orange line, 0.1m thick
  marker.color.r = 1.0; marker.color.g = 0.36; marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;

  // Add path points
  for (const auto& point : path.path) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.0;
    marker.points.push_back(p);
  }
  
  final_path_vis_pub_->publish(marker);
}

void LocalPlannerNode::publish_final_path(const Path & path){
nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";
  
  for(const auto & pt : path.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;
    
    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta);
    pose.pose.orientation = tf2::toMsg(q);
    
    path_msg.poses.push_back(pose);
  }
  
  path_pub_->publish(path_msg);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Local Planning node");
  planned_path_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(planned_paths_vis_topic, 10);
  final_path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(final_path_vis_topic, 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(final_path_topic, 10);

  RCLCPP_INFO(this->get_logger(), "Node configured successfully");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Local Planning node");
  lanelet_ahead_sub_ = create_subscription<lanelet_msgs::msg::LaneletAhead>(lanelet_ahead_topic, 10, std::bind(&LocalPlannerNode::lanelet_update_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&LocalPlannerNode::update_vehicle_odom, this, std::placeholders::_1));
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10, std::bind(&LocalPlannerNode::update_costmap, this, std::placeholders::_1));
  bt_sub_ = create_subscription<behaviour_msgs::msg::ExecuteBehaviour>(bt_topic, 10, std::bind(&LocalPlannerNode::set_preferred_lanes, this, std::placeholders::_1));
  planned_path_vis_pub_->on_activate();   
  final_path_vis_pub_->on_activate();   
  path_pub_->on_activate();   
  RCLCPP_INFO(this->get_logger(), "Node Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Local Planning node");
  lanelet_ahead_sub_.reset();
  costmap_sub_.reset();
  bt_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Local Planning node");
  lanelet_ahead_sub_.reset();
  costmap_sub_.reset();
  bt_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LocalPlannerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Local Planning node");
  lanelet_ahead_sub_.reset();
  costmap_sub_.reset();
  bt_sub_.reset();
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