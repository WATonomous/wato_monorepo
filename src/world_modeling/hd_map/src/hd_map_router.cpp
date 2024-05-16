#include "hd_map_router.hpp"

HDMapRouter::HDMapRouter() {}

lanelet::GPSPoint ros_gps_msg_to_lanelet_gps_point(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
    lanelet::GPSPoint gpsPoint = lanelet::GPSPoint();
    gpsPoint.lat = gps_msg->latitude;
    gpsPoint.lon = gps_msg->longitude;
    gpsPoint.ele = gps_msg->altitude;

    return gpsPoint;
}

bool HDMapRouter::set_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr){
    lanelet::routing::RoutingGraph::Errors errors;
    this->lanelet_ptr_ = lanelet_ptr;

    lanelet::traffic_rules::TrafficRulesPtr traffic_rules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr routing_graph = lanelet::routing::RoutingGraph::build(*this->lanelet_ptr_, *traffic_rules);
    
    this->routing_graph_ = routing_graph;
    
    errors = routing_graph_->checkValidity();
    if (errors.empty() != true){
            RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Assigning Routing Graph... Failed");
            for (auto error : errors){
                RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Routing Graph Build Error : %s", error.c_str());
            }
            return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Building Traffic Rules Factory... Success");
    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Assigning Routing Graph... Success");

    return true;
}

lanelet::LaneletMapPtr HDMapRouter::get_lanelet(){
    return this->lanelet_ptr_;
}

bool HDMapRouter::set_projector(std::shared_ptr<lanelet::Projector> projector){
    this->projector_ = projector;
    return true;
}

lanelet::BasicPoint3d HDMapRouter::project_gps_to_point3d(lanelet::GPSPoint gps_point){
    return projector_->forward(gps_point);
}

lanelet::GPSPoint HDMapRouter::project_point3d_to_gps(lanelet::BasicPoint3d point3d){
    return projector_->reverse(point3d);
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_gps(lanelet::GPSPoint gps_point){
    auto search_point = project_gps_to_point3d(gps_point);
    double min_distance = __DBL_MAX__;
    lanelet::ConstLanelet nearest_lanelet;

    for (const auto &lanelet : lanelet_ptr_->laneletLayer){
        double distance = lanelet::geometry::distanceToCenterline3d(lanelet, search_point);
        if (distance < min_distance){
            min_distance = distance;
            nearest_lanelet = lanelet;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Found closest lanelet to GPS Point!");
    return nearest_lanelet;
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_xy(float x, float y, float width_x, float height_y){
    float x_center, y_center;

    x_center = (x + (0.5)*width_x);
    y_center = (y - (0.5)*height_y);

    lanelet::BasicPoint3d obstacle_center_xy(x_center, y_center, 0);
    lanelet::GPSPoint local_coordinates_xy = projector_->reverse(obstacle_center_xy);

    return get_nearest_lanelet_to_gps(local_coordinates_xy);
}

lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_xyz(float x_center, float y_center, float z_center){
    lanelet::BasicPoint3d obstacle_center_xyz(x_center, y_center, z_center);
    lanelet::GPSPoint local_coordinates_xyz = projector_->reverse(obstacle_center_xyz);

    return get_nearest_lanelet_to_gps(local_coordinates_xyz);
}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point){
    return route(get_nearest_lanelet_to_gps(from_point), get_nearest_lanelet_to_gps(to_point));
}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet){
    auto route = this->routing_graph_->getRoute(from_lanelet, to_lanelet, 0);
    auto shortest_path = route->shortestPath();
    return shortest_path;
}

// Obstacle Message : https://github.com/WATonomous/wato_monorepo/blob/32946e5cbbc1721d404aa4851d58c7425b8121bc/src/wato_msgs/common_msgs/msg/Obstacle.msg
void HDMapRouter::process_obstacle_msg(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    if (!obstacle_msg_ptr){
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Obstacle message is empty!");
        return;
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Obstacle message retrieved!");

        if (obstacle_list.find(obstacle_msg_ptr->object_id) == obstacle_list.end()){
            RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "New Obstacle! Adding Element to the HD-Map...");
            add_obstacle(obstacle_msg_ptr);
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Obstacle Exists in Map! Updating Obstacle info...");
            update_obstacle(obstacle_msg_ptr);            
        }
    }
}

void HDMapRouter::add_obstacle(common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    bool result = false;

    if (obstacle_msg_ptr->label == "STOP SIGN"){
        result = add_stop_sign_reg_elem(obstacle_msg_ptr);
    }
    else if (obstacle_msg_ptr->label == "TRAFFIC LIGHT"){
        result = add_traffic_light_reg_elem(obstacle_msg_ptr);
    }
    else if (obstacle_msg_ptr->label == "PEDESTRIAN"){
        result = add_pedestrian_reg_elem(obstacle_msg_ptr);
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Obstacle Message %s not recognized!", obstacle_msg_ptr->label.c_str());
    } 
}

// TODO : update_obstacle_msg()
// DESCRIPTION : 
//          To update the properties of the obstacle message on the HD-Map
void HDMapRouter::update_obstacle(common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){ 
}


// Old implementation: https://github.com/WATonomous/wato_monorepo_autodrive/blob/develop/src/path_planning/env_model/src/
bool HDMapRouter::add_stop_sign_reg_elem(common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    // TODO : stop sign
}

bool HDMapRouter::add_pedestrian_reg_elem(common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    // TODO : pedestrian
}

bool HDMapRouter::add_traffic_light_reg_elem(common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    // TODO : traffic light
}
