// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WORLD_MODELING_MAP_MANAGER_
#define WORLD_MODELING_MAP_MANAGER_

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <map>
#include <memory>
#include <string>

#include "hd_map/hd_map_router.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class HDMapManager
{
public:
  explicit HDMapManager(std::shared_ptr<HDMapRouter> router);

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

  /**
  * Get the router instance used by this manager.
  *
  * @return shared pointer to the HDMapRouter owned by the manager.
  */
  std::shared_ptr<HDMapRouter> get_router() const
  {
    return router_;
  }

  /**
  * Get the base directory where OSM maps are stored.
  *
  * @return absolute path to the maps directory.
  */
  std::string get_maps_directory() const;

private:
  /**
   * Project the OSM map to Lanelet format using the specified origin.
   *
   * @param filename the name of the OSM file
   * @param origin the origin for the projection
   * @param lanelet_ptr the LaneletMapPtr where the result is stored
   * @return whether the projection was successful
   */
  bool project_osm_to_lanelet(std::string filename, lanelet::Origin origin, lanelet::LaneletMapPtr & lanelet_ptr);

  /**
   * Project the OSM map to Lanelet format using the specified projector.
   *
   * @param filename the name of the OSM file
   * @param projector the projector to use for the transformation
   * @param lanelet_ptr the LaneletMapPtr where the result is stored
   * @return whether the projection was successful
   */
  bool project_osm_to_lanelet(
    std::string filename, const lanelet::Projector & projector, lanelet::LaneletMapPtr & lanelet_ptr);

  /**
   * Set the Lanelet map in the router.
   *
   * @param lanelet_ptr the Lanelet map to set
   * @return whether the map was successfully set
   */
  bool set_map_router_lanelet(const lanelet::LaneletMapPtr & lanelet_ptr);

  /**
   * Retrieve the origin for a specific map file.
   *
   * @param filename the filename of the OSM map
   * @param origin the origin for the map
   * @return whether the origin was successfully retrieved
   */
  bool get_origin_from_filename(std::string filename, lanelet::Origin & origin);

  std::string maps_directory_ = "/home/bolty/ament_ws/etc/maps/osm/";
  std::map<std::string, lanelet::Origin> originList;
  std::map<std::string, lanelet::Origin> create_origin_list() const;

  std::shared_ptr<HDMapRouter> router_;
};
#endif
