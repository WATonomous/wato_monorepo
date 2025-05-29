#ifndef WORLD_MODELING_MAP_MANAGER_
#define WORLD_MODELING_MAP_MANAGER_

#include "rclcpp/rclcpp.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

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
   * @param gps_msg a sensor_msgs::msg::NavSatFix message
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
   * @param gps_msg a sensor_msgs::msg::NavSatFix message
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
  /**
   * Project the OSM map to Lanelet format using the specified origin.
   *
   * @param filename the name of the OSM file
   * @param origin the origin for the projection
   * @param lanelet_ptr the LaneletMapPtr where the result is stored
   * @return whether the projection was successful
   */
  bool project_osm_to_lanelet(std::string filename, lanelet::Origin origin,
                              lanelet::LaneletMapPtr &lanelet_ptr);

  /**
   * Project the OSM map to Lanelet format using the specified projector.
   *
   * @param filename the name of the OSM file
   * @param projector the projector to use for the transformation
   * @param lanelet_ptr the LaneletMapPtr where the result is stored
   * @return whether the projection was successful
   */
  bool project_osm_to_lanelet(std::string filename, const lanelet::Projector &projector,
                              lanelet::LaneletMapPtr &lanelet_ptr);

  /**
   * Set the Lanelet map in the router.
   *
   * @param lanelet_ptr the Lanelet map to set
   * @return whether the map was successfully set
   */
  bool set_map_router_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr);

  /**
   * Retrieve the origin for a specific map file.
   *
   * @param filename the filename of the OSM map
   * @param origin the origin for the map
   * @return whether the origin was successfully retrieved
   */
  bool get_origin_from_filename(std::string filename, lanelet::Origin &origin);

  // List of origins associated with map files
  std::map<std::string, lanelet::Origin> originList = {
      {"/home/bolty/ament_ws/etc/maps/osm/Town05.osm", lanelet::Origin({0, 0})}};

  // Shared pointer to the HDMapRouter object
  std::shared_ptr<HDMapRouter> router_;
};

#endif
