#include "hd_map_manager.hpp"

#include <iostream>

HDMapManager::HDMapManager(std::shared_ptr<HDMapRouter> router) : router_(router) {}

/**
 * Select an osm map from the maps directory given the filename.
 * 
 * @param filename the filename of the map
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map(std::string filename) {
    lanelet::Origin origin;
    // Get origin of lanelet
    if(get_origin_from_filename(filename, origin)){
        lanelet::LaneletMapPtr lanelet_ptr;
        auto projector = std::make_shared<lanelet::projection::UtmProjector>(origin);
        // Project osm to lanelet with origin
        if(project_osm_to_lanelet(filename, *projector, lanelet_ptr)){
            // Set map and return set status
            return set_map_router_lanelet(lanelet_ptr) && router_->set_projector(projector);
        }
    }

    // Intermediate step failed, return false
    return false;
}

/**
 * Select an osm map from the maps directory which falls within the coordinates specified.
 *
 * This will first get the map and then set it as the current lanelet in the HDMapRouter.
 * 
 * @param gps_point a sensor_msgs::msg::NavSatFix msg
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
    return select_osm_map(get_osm_map_from_coordinates(gps_msg));
}

/**
 * Select an osm map from the maps directory which falls within the coordinates specified.
 *
 * This will first get the map and then set it as the current lanelet in the HDMapRouter.
 * 
 * @param gps_point a lanelet::GPSPoint object
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map_from_coordinates(lanelet::GPSPoint gps_point) {
    return select_osm_map(get_osm_map_from_coordinates(gps_point));
}

/**
 * Get an osm map from the maps directory which falls within the coordinates specified.
 * 
 * @param gps_point a sensor_msgs::msg::NavSatFix message
 * @return the file name of the lanelet within the gps_msg
 */
std::string HDMapManager::get_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
    return get_osm_map_from_coordinates(ros_gps_msg_to_lanelet_gps_point(gps_msg));
}

struct OSMMap {
    std::string filename;
    double min_latitude;
    double max_latitude;
    double min_longitude;
    double max_longitude;
};

/**
 * Get an osm map from the maps directory which falls within the coordinates specified.
 * 
 * @param gps_point a lanelet::GPSPoint object
 * @return the file name of the lanelet within the gps_point
 */
std::string HDMapManager::get_osm_map_from_coordinates(lanelet::GPSPoint gps_point) {
    // TODO: Implement
    // Each map will have GPS bounds (need to initialize list and hard-code bounds of maps for now)
    // Search through lists and find map which gps point is within bounds
    // return the filename of that map, empty string for no map
    std::unordered_map<std::string, OSMMap> osm_maps;

    // Initialize example maps into the hashmap
        osm_maps = {
            {{"map1.osm", 40.0, 41.0, -75.0, -74.0}},
            {{"map2.osm", 42.0, 43.0, -76.0, -75.0}},       
            // ...
        };

    // go through the maps and see if any of them are within the range
    auto itr = osm_maps.begin();
    while (itr != osm_maps.end()) {
        if  (gps_point.latitude() >= itr->second.min_latitude &&
             gps_point.latitude() <= itr->second.max_latitude &&
             gps_point.longitude() >= itr->second.min_longitude &&
             gps_point.longitude() <= itr->second.max_longitude){
             return itr->map.filename;
        }
        itr++;
    }

    // return empty
    return "";    
}

bool HDMapManager::project_osm_to_lanelet(std::string filename, lanelet::Origin origin, lanelet::LaneletMapPtr &lanelet_ptr) {
    return project_osm_to_lanelet(filename, lanelet::projection::UtmProjector(origin), lanelet_ptr);
}

bool HDMapManager::project_osm_to_lanelet(std::string filename, const lanelet::Projector &projector, lanelet::LaneletMapPtr &lanelet_ptr) {
    auto lanelet = lanelet::load(filename.c_str(), projector);
    lanelet_ptr = std::move(lanelet);
    return true;
}

bool HDMapManager::set_map_router_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr){
    return router_->set_lanelet(lanelet_ptr);
}

bool HDMapManager::get_origin_from_filename(std::string filename, lanelet::Origin &origin){
    if (originList.find(filename) == originList.end()) {
        return false;
    }
    origin = originList[filename];
    return true;
}
