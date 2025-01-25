#include <memory>

#include "hd_map_service.hpp"

HDMapService::HDMapService() : Node("hd_map_service") {
  // Declare parameters
  this->declare_parameter<std::string>("vizualization_output_topic", std::string("hd_map"));
  this->declare_parameter<std::string>("route_output_topic", std::string("hd_map_route"));
  this->declare_parameter<std::string>("start_output_topic", std::string("hd_map_start_lanelet"));
  this->declare_parameter<std::string>("end_output_topic", std::string("hd_map_end_lanelet"));
  this->declare_parameter<std::string>("desired_lane_output_topic", std::string("hd_map_desired_lane"));
  this->declare_parameter<std::string>("current_lane_output_topic", std::string("hd_map_current_lane"));

  this->declare_parameter<std::string>("traffic_light_input_topic", std::string("traffic_light"));
  this->declare_parameter<std::string>("traffic_sign_input_topic", std::string("traffic_sign"));
  this->declare_parameter<std::string>("pedestrian_input_topic", std::string("pedestrian"));
  this->declare_parameter<std::string>("point_input_topic", std::string("clicked_point"));
  this->declare_parameter<std::string>("query_point_input_topic", std::string("query_point"));

  // Get parameters
  std::string vizualization_output_topic = this->get_parameter("vizualization_output_topic").as_string();
  std::string route_output_topic = this->get_parameter("route_output_topic").as_string();
  std::string start_output_topic = this->get_parameter("start_output_topic").as_string();
  std::string end_output_topic = this->get_parameter("end_output_topic").as_string(); 
  std::string desired_lane_output_topic = this->get_parameter("desired_lane_output_topic").as_string();
  std::string current_lane_output_topic = this->get_parameter("current_lane_output_topic").as_string();
  
  std::string traffic_light_input_topic = this->get_parameter("traffic_light_input_topic").as_string();
  std::string traffic_sign_input_topic = this->get_parameter("traffic_sign_input_topic").as_string();
  std::string pedestrian_input_topic = this->get_parameter("pedestrian_input_topic").as_string();
  std::string point_input_topic = this->get_parameter("point_input_topic").as_string();
  std::string query_point_input_topic = this->get_parameter("query_point_input_topic").as_string();


  router_ = std::make_shared<HDMapRouter>();
  manager_ = std::make_shared<HDMapManager>(this->router_);

  behaviour_tree_info_service = this->create_service<world_modeling_msgs::srv::BehaviourTreeInfo>("behaviour_tree_info", std::bind(&HDMapService::behaviour_tree_info_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Map selection hardcoded for now
  RCLCPP_INFO(this->get_logger(), "Selecting Lanelet Map Town05.osm...\n");
  if (manager_->select_osm_map("/home/bolty/ament_ws/etc/maps/osm/Town05.osm")) {
    RCLCPP_INFO(this->get_logger(), "Map Selection Successful!\n");
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Map Selection Failed.\n");
  }

  hd_map_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(vizualization_output_topic, 20);
  hd_map_route_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(route_output_topic, 20);
  hd_map_start_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(start_output_topic, 20);
  hd_map_end_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(end_output_topic, 20);
  hd_map_desired_lane_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(desired_lane_output_topic, 20);
  hd_map_current_lane_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(current_lane_output_topic, 20);

  hd_map_traffic_light_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(traffic_light_input_topic, 20, std::bind(&HDMapService::hd_map_traffic_light_callback, this, std::placeholders::_1));
  hd_map_traffic_sign_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3D>(traffic_sign_input_topic, 20, std::bind(&HDMapService::hd_map_traffic_sign_callback, this, std::placeholders::_1));
  hd_map_pedestrian_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(pedestrian_input_topic, 20, std::bind(&HDMapService::hd_map_pedestrian_callback, this, std::placeholders::_1));
  point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(point_input_topic, 20, std::bind(&HDMapService::point_callback, this, std::placeholders::_1));
  query_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(query_point_input_topic, 20, std::bind(&HDMapService::get_desired_lane, this, std::placeholders::_1));

  hd_map_visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&HDMapService::publish_hd_map_marker, this));
}

void HDMapService::hd_map_traffic_light_callback(vision_msgs::msg::Detection3DArray::SharedPtr traffic_light_array_msg) {
  router_->process_traffic_light_msg(traffic_light_array_msg);
}

void HDMapService::hd_map_traffic_sign_callback(vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg) {
  router_->process_traffic_sign_msg(traffic_sign_msg);
}

void HDMapService::hd_map_pedestrian_callback(vision_msgs::msg::Detection3DArray::SharedPtr pedestrian_msg) {
  router_->process_pedestrian_msg(pedestrian_msg);
}

void HDMapService::publish_hd_map_marker(){
  auto marker_array = world_modeling::hd_map::laneletMapAsMarkerArray(router_->get_lanelet());

  RCLCPP_INFO(this->get_logger(), "Publishing Lanelet Message from HD Map...\n");
  hd_map_visualization_publisher_->publish(marker_array);
}

void HDMapService::point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point_ = msg->point;

  auto pt1 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{0, 0, 0});
  auto pt2 = router_->project_point3d_to_gps(lanelet::BasicPoint3d{msg->point.x, msg->point.y, msg->point.z});
  
  RCLCPP_INFO(this->get_logger(), "GPS Point 1: %f %f %f", pt1.lat, pt1.lon, pt1.ele);
  RCLCPP_INFO(this->get_logger(), "GPS Point 2: %f %f %f", pt2.lat, pt2.lon, pt2.ele);

  auto lanelet1 = router_->get_nearest_lanelet_to_gps(pt1);
  auto lanelet2 = router_->get_nearest_lanelet_to_gps(pt2);
  
  std_msgs::msg::ColorRGBA color;
  color.g = 1;
  color.a = 1;

  int id1 = 0;

  auto marker1 = world_modeling::hd_map::laneletAsMarkerArray(lanelet1, &id1, false, true, color, color);
  hd_map_start_publisher_->publish(marker1);
  int id2 = 0;
  auto marker2 = world_modeling::hd_map::laneletAsMarkerArray(lanelet2, &id2, false, true, color, color);
  hd_map_end_publisher_->publish(marker2);

  lanelet_path = router_->route(pt1, pt2);
  if (lanelet_path) {
    RCLCPP_INFO(this->get_logger(), "Route Found. Publishing Route Markers...\n");
    for(auto lanelet = lanelet_path->begin(); lanelet != lanelet_path->end(); ++lanelet) {
        RCLCPP_INFO(this->get_logger(), "Lanelet: \n");
    }
    auto path_marker_array = world_modeling::hd_map::laneletPathAsMarkerArray(*lanelet_path);
    hd_map_route_publisher_->publish(path_marker_array);
  }
}

void HDMapService::get_desired_lane(geometry_msgs::msg::PointStamped::SharedPtr msg) {
  current_point_ = msg->point;

  auto pt = router_->project_point3d_to_gps(lanelet::BasicPoint3d{msg->point.x, msg->point.y, msg->point.z});
  auto lanelet = router_->get_nearest_lanelet_to_gps(pt);

  current_lanelet_ = lanelet;

  if(lanelet_path) {
    auto it = std::find(lanelet_path->begin(), lanelet_path->end(), lanelet);
    if (it != lanelet_path->end()) {
      int idx = it-lanelet_path->begin();
      RCLCPP_INFO(this->get_logger(), "Found lanelet: %i\n", idx);

      visualization_msgs::msg::MarkerArray current_marker;
      visualization_msgs::msg::MarkerArray desired_marker;
      std_msgs::msg::ColorRGBA color;
      color.b = 1;
      color.a = 1;
      int id = 0;
      if (lanelet_path->getRemainingLane(it).size() > 1) {
        RCLCPP_INFO(this->get_logger(), "Using Current It");
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
      }
      else if (idx < lanelet_path->size()-1) {
        RCLCPP_INFO(this->get_logger(), "Using Next It");
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray((*lanelet_path)[idx+1], &id, false, true, color, color, .3, .4);
      }
      else {
        current_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
        desired_marker = world_modeling::hd_map::laneletAsMarkerArray(*it, &id, false, true, color, color, .3, .4);
      }

      hd_map_desired_lane_publisher_->publish(desired_marker);
      hd_map_current_lane_publisher_->publish(current_marker);
    }
  }
}

// TODO convert lanelet to lanelet msg

world_modeling_msgs::msg::Lanelet HDMapService::convert_lanelet_to_msg(const lanelet::ConstLanelet& lanelet){
    world_modeling_msgs::msg::Lanelet lanelet_msg;

    //convert left boundary
    for (const auto& point : lanelet.leftBound()){
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      lanelet_msg.left_boundary.push_back(p);
    }

    //convert right boundary
    for (const auto& point : lanelet.rightBound()){
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      lanelet_msg.left_boundary.push_back(p);
    }
    
    //convert centerline
    for (const auto& point : lanelet.centerline()) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        lanelet_msg.centerline.push_back(p);
    }

    lanelet_msg.id = lanelet.id();

    return lanelet_msg;
}

// TODO convert path to lanelet path msg

world_modeling_msgs::msg::LaneletPath HDMapService::convert_laneletPath_to_msg(const lanelet::Optional<lanelet::routing::LaneletPath>& path){
  world_modeling_msgs::msg::LaneletPath path_msg;
  
  if (path) {
    for (const auto& lanelet : *path) {
      path_msg.lanelets.push_back(this->convert_lanelet_to_msg(lanelet));
    }
  }

  return path_msg;
}

void HDMapService::behaviour_tree_info_callback(const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Request> request, const std::shared_ptr<world_modeling_msgs::srv::BehaviourTreeInfo::Response> response){
  response->current_point = current_point_;
  response->goal_point = goal_point_; 

  // response->current_lanelet = convert_lanelet_to_msg(current_lanelet_);
  // response->goal_lanelet = convert_lanelet_to_msg(goal_lanelet_);
  // response->route_list = convert_laneletPath_to_msg(lanelet_path);
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HDMapService>());
  rclcpp::shutdown();
  return 0;
}
