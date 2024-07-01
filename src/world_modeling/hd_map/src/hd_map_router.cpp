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

void HDMapRouter::process_traffic_light_msg(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr){
    uint64_t traffic_light_id = stoi(traffic_light_msg_ptr->id);
    if(traffic_light_list_.find(traffic_light_id) == nullptr){
        add_traffic_light(traffic_light_msg_ptr);
        traffic_light_list_.insert(traffic_light_id);
    }
    else{
        update_traffic_light(traffic_light_msg_ptr);
    }
}

void HDMapRouter::process_traffic_sign_msg(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){
    uint64_t traffic_sign_id = stoi(traffic_sign_msg_ptr->id);
    if(traffic_sign_list_.find(traffic_sign_id) == nullptr){
        add_traffic_sign(traffic_sign_msg_ptr);
        traffic_sign_list_.insert(traffic_sign_id);
    }
    else{
        update_traffic_sign(traffic_sign_msg_ptr);
    }
}

void HDMapRouter::process_obstacle_msg(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    uint32_t obstacle_id = obstacle_msg_ptr->object_id;
    if(obstacle_list_.find(obstacle_id) == nullptr){
        add_obstacle(obstacle_msg_ptr);
        obstacle_list_.insert(obstacle_id);
    }
    else{
        update_obstacle(obstacle_msg_ptr);
    }
}

std::string HDMapRouter::get_detection3d_class(const vision_msgs::msg::Detection3D::SharedPtr reg_elem_msg_ptr){
    std::string class_id = "";
    float base_score = 0;
    for (const auto result : reg_elem_msg_ptr->results){
        if (result.hypothesis.score > base_score){
            class_id = result.hypothesis.class_id;
        }
    }
    return class_id;
}

// Update Regulatory Element Functions:
//      - update_traffic_light() [TODO]
//      - update_traffic_sign() [DONE]
//          A. update_stop_sign()  [TODO]
//             ... (more to come later, hopefully)  
//      - update_obstacle() [TODO]
void HDMapRouter::update_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_light_msg_ptr){

}

void HDMapRouter::update_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){
    std::string traffic_sign_name = HDMapRouter::get_detection3d_class(traffic_sign_msg_ptr);
    if (traffic_sign_name == "STOP SIGN"){
        update_stop_sign(traffic_sign_msg_ptr);
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Sign Type Does Not Exist in Vocabulary!");
    }
}

void HDMapRouter::update_stop_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){

}

void HDMapRouter::update_obstacle(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){

}

// Add Regulatory Element Functions:
//      - add_traffic_light() [TODO]
//      - add_traffic_sign() [DONE]
//          A. add_stop_sign()  [TODO]
//             ... (more to come later, hopefully)  
//      - add_obstacle() [TODO]
void HDMapRouter::add_traffic_light(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){
    // TODO : traffic light
}

void HDMapRouter::add_traffic_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){
    std::string traffic_sign_name = HDMapRouter::get_detection3d_class(traffic_sign_msg_ptr);
    if (traffic_sign_name == "STOP SIGN"){
        add_stop_sign(traffic_sign_msg_ptr);
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Traffic Sign Type Does Not Exist in Vocabulary!");
    }
}
void HDMapRouter::add_stop_sign(const vision_msgs::msg::Detection3D::SharedPtr traffic_sign_msg_ptr){
    // TODO : stop sign
}

void HDMapRouter::add_obstacle(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    // TODO : pedestrian
}
