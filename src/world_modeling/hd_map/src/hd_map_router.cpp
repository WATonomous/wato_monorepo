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

// TODO: implementation of get nearest lanelet to Obstacle x, y, z coordinates
// Reference: [FINDING]
lanelet::ConstLanelet HDMapRouter::get_nearest_lanelet_to_xyz(float x, float y, float z){

}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(lanelet::GPSPoint from_point, lanelet::GPSPoint to_point){
    return route(get_nearest_lanelet_to_gps(from_point), get_nearest_lanelet_to_gps(to_point));
}

lanelet::Optional<lanelet::routing::LaneletPath> HDMapRouter::route(lanelet::ConstLanelet from_lanelet, lanelet::ConstLanelet to_lanelet){
    auto route = this->routing_graph_->getRoute(from_lanelet, to_lanelet, 0);
    auto shortest_path = route->shortestPath();
    return shortest_path;
}

void HDMapRouter::process_obstacle_msg(const common_msgs::msg::Obstacle::SharedPtr obstacle_msg_ptr){
    if (!obstacle_msg_ptr){
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Obstacle message is empty!");
        return;
    }

    std::string obstacle_label = obstacle_msg_ptr->label;

    geometry_msgs::msg::PoseWithCovariance obstacle_pose_covariance = obstacle_msg_ptr->pose;
    geometry_msgs::msg::Pose obstacle_pose = obstacle_pose_covariance.pose;
    geometry_msgs::msg::Point obstacle_point = obstacle_pose.position;
    float x = obstacle_point.x;
    float y = obstacle_point.y;
    float z = obstacle_point.z;

    float width_x = obstacle_msg_ptr->width_along_x_axis;
    float height_y = obstacle_msg_ptr->height_along_y_axis;
    float depth_z = obstacle_msg_ptr->depth_along_z_axis;

    uint32_t object_id = obstacle_msg_ptr->object_id;

    RCLCPP_INFO(rclcpp::get_logger("hd_map_router"), "Obstacle message retrieved!");
    
    lanelet::ConstLanelet reg_elem_lanelet = HDMapRouter::get_nearest_lanelet_to_xyz(x, y, z);

    if (obstacle_label == "STOP SIGN"){
        add_stop_sign_reg_elem(reg_elem_lanelet);
    }
    else if (obstacle_label == "PEDESTRIAN"){
        add_pedestrian_reg_elem(reg_elem_lanelet);
    }
    else if (obstacle_label == "TRAFFIC LIGHT"){
        add_traffic_light_reg_elem(reg_elem_lanelet);
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("hd_map_router"), "Recieved obstacle label has no associated RegElem!");
    }
}


// TODO: functions to add the three regulatory elements on the DRG
// Old implementation: https://github.com/WATonomous/wato_monorepo_autodrive/blob/develop/src/path_planning/env_model/src/
void HDMapRouter::add_stop_sign_reg_elem(lanelet::ConstLanelet reg_elem_lanelet){
    // TODO : stop sign
}

void HDMapRouter::add_pedestrian_reg_elem(lanelet::ConstLanelet reg_elem_lanelet){
    // TODO : pedestrian
}

void HDMapRouter::add_traffic_light_reg_elem(lanelet::ConstLanelet reg_elem_lanelet){
    // TODO : traffic light
}
