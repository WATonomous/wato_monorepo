#ifndef WORLD_MODELING_MAP_MANAGER_
#define WORLD_MODELING_MAP_MANAGER_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "hd_map_router.hpp"

class HDMapManager {
  public:
    HDMapManager(std::shared_ptr<HDMapRouter> router);

    /**
     * Select an osm map from the maps directory given the filename.
     * 
     * @param filename the filename of the map
     * @return whether the selection was successful
     */
    bool select_osm_map(std::string filename);
    
    /**
     * Select an osm map from the maps directory which falls within the coordinates specified.
     *
     * This will first get the map and then set it as the current lanelet in the HDMapRouter.
     * 
     * @param gps_point a sensor_msgs::msg::NavSatFix msg
     * @return whether the selection was successful
     */
    bool select_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

    /**
     * Select an osm map from the maps directory which falls within the coordinates specified.
     *
     * This will first get the map and then set it as the current lanelet in the HDMapRouter.
     * 
     * @param gps_point a lanelet::GPSPoint object
     * @return whether the selection was successful
     */
    bool select_osm_map_from_coordinates(lanelet::GPSPoint gps_point);

    /**
     * Get an osm map from the maps directory which falls within the coordinates specified.
     * 
     * @param gps_point a sensor_msgs::msg::NavSatFix message
     * @return the file name of the lanelet within the gps_msg
     */
    std::string get_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
    
    /**
     * Get an osm map from the maps directory which falls within the coordinates specified.
     * 
     * @param gps_point a lanelet::GPSPoint object
     * @return the file name of the lanelet within the gps_point
     */
    std::string get_osm_map_from_coordinates(lanelet::GPSPoint gps_point);

  private:
    bool project_osm_to_lanelet(std::string filename, lanelet::Origin origin, lanelet::LaneletMapPtr &lanelet_ptr);
    bool project_osm_to_lanelet(std::string filename, const lanelet::Projector &projector, lanelet::LaneletMapPtr &lanelet_ptr);
    
    bool set_map_router_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr);

    bool get_origin_from_filename(std::string filename, lanelet::Origin &origin);

    std::map<std::string, lanelet::Origin> originList = {
        {"/home/bolty/ament_ws/etc/maps/osm/Town05.osm", lanelet::Origin({0, 0})}
    };

    std::shared_ptr<HDMapRouter> router_;
};

#endif
