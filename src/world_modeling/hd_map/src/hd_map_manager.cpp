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

#include "hd_map/hd_map_manager.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

HDMapManager::HDMapManager(std::shared_ptr<HDMapRouter> router)
    : router_(router), originList(create_origin_list())
{
}

/**
 * Select an OSM map from the maps directory given the filename.
 *
 * @param filename the filename of the map
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map(std::string filename)
{
  lanelet::Origin origin;

  // Retrieve the origin from the filename
  if (get_origin_from_filename(filename, origin))
  {
    lanelet::LaneletMapPtr lanelet_ptr;
    auto projector = std::make_shared<lanelet::projection::UtmProjector>(origin);
    // Project the OSM map to a Lanelet map using the origin
    if (project_osm_to_lanelet(filename, *projector, lanelet_ptr))
    {
      // Set the map and return the status of the set operation
      return set_map_router_lanelet(lanelet_ptr) && router_->set_projector(projector);
    }
  }

  return false;
}

/**
 * Select an OSM map from the maps directory based on coordinates from a NavSatFix message.
 *
 * This function will fetch the map and then set it as the current lanelet in the HDMapRouter.
 *
 * @param gps_msg a sensor_msgs::msg::NavSatFix message
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
  return select_osm_map(get_osm_map_from_coordinates(gps_msg));
}

/**
 * Select an OSM map from the maps directory based on coordinates from a GPSPoint.
 *
 * This function will fetch the map and then set it as the current lanelet in the HDMapRouter.
 *
 * @param gps_point a lanelet::GPSPoint object
 * @return whether the selection was successful
 */
bool HDMapManager::select_osm_map_from_coordinates(lanelet::GPSPoint gps_point)
{
  return select_osm_map(get_osm_map_from_coordinates(gps_point));
}

/**
 * Get the filename of the OSM map corresponding to the specified GPS coordinates (from a NavSatFix
 * message).
 *
 * @param gps_msg a sensor_msgs::msg::NavSatFix message
 * @return the filename of the OSM map
 */
std::string HDMapManager::get_osm_map_from_coordinates(sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
  return get_osm_map_from_coordinates(ros_gps_msg_to_lanelet_gps_point(gps_msg));
}

std::string HDMapManager::get_maps_directory() const { return maps_directory_; };

struct OSMMap
{
  std::string filename;
  double min_latitude;
  double max_latitude;
  double min_longitude;
  double max_longitude;
};

/**
 * Get the filename of the OSM map corresponding to the specified GPS coordinates (from a GPSPoint).
 *
 * @param gps_point a lanelet::GPSPoint object
 * @return the filename of the OSM map
 */
std::string HDMapManager::get_osm_map_from_coordinates(lanelet::GPSPoint gps_point)
{
  // TODO: Store this somewhere else, maybe we should have a JSON file in the map_data repo with all the relevant coords
  std::vector<OSMMap> osm_maps = {
      {"singapore-queenstown.osm",
       1.28576172821,
       1.30983579179,
       103.77006336732,
       103.79522411848},
      {"boston-seaport.osm",
       42.33802019067,
       42.35523092991,
       -71.05709479455,
       -71.02214424789},
      {"singapore-hollandvillage.osm",
       1.30645855219,
       1.32556886974,
       103.78568943976,
       103.80529183610},
      {"singapore-onenorth.osm",
       1.28888635829,
       1.30586349097,
       103.78494564386,
       103.79847124916}};

  // Example of how you might use it:
  for (const auto &map : osm_maps)
  {
    if (gps_point.lat >= map.min_latitude && gps_point.lat <= map.max_latitude &&
        gps_point.lon >= map.min_longitude && gps_point.lon <= map.max_longitude)
    {
      return map.filename;
    }
  }

  return "";
}

/**
 * Project the OSM map to a Lanelet map using the given origin.
 *
 * @param filename the OSM map filename
 * @param origin the origin of the map
 * @param lanelet_ptr reference to the Lanelet map pointer
 * @return whether the projection was successful
 */
bool HDMapManager::project_osm_to_lanelet(
    std::string filename, lanelet::Origin origin, lanelet::LaneletMapPtr &lanelet_ptr)
{
  return project_osm_to_lanelet(filename, lanelet::projection::UtmProjector(origin), lanelet_ptr);
}

/**
 * Project the OSM map to a Lanelet map using the specified projector.
 *
 * @param filename the OSM map filename
 * @param projector the projector to use
 * @param lanelet_ptr reference to the Lanelet map pointer
 * @return whether the projection was successful
 */
bool HDMapManager::project_osm_to_lanelet(
    std::string filename, const lanelet::Projector &projector, lanelet::LaneletMapPtr &lanelet_ptr)
{
  auto lanelet = lanelet::load(filename.c_str(), projector);
  lanelet_ptr = std::move(lanelet);
  return true;
}

/**
 * Set the Lanelet map for the HDMapRouter.
 *
 * @param lanelet_ptr the Lanelet map pointer
 * @return whether the map was set successfully
 */
bool HDMapManager::set_map_router_lanelet(const lanelet::LaneletMapPtr &lanelet_ptr)
{
  return router_->set_lanelet(lanelet_ptr);
}

/**
 * Get the origin associated with the given filename.
 *
 * @param filename the OSM map filename
 * @param origin reference to the origin object
 * @return whether the origin was found
 */
bool HDMapManager::get_origin_from_filename(std::string filename, lanelet::Origin &origin)
{
  if (originList.find(filename) == originList.end())
  {
    return false;
  }
  origin = originList[filename];
  return true;
}

std::map<std::string, lanelet::Origin> HDMapManager::create_origin_list() const
{
  static const std::vector<std::pair<std::string, lanelet::Origin>> map_origins = {
      {"Town05.osm", lanelet::Origin({0, 0})},
      {"Town10HD.osm", lanelet::Origin({0, 0})},
      {"boston-seaport.osm", lanelet::Origin({42.34662556029, -71.03961952122})},
      {"singapore-onenorth.osm", lanelet::Origin({1.29737492463, 103.79170844651})},
      {"singapore-hollandvillage.osm", lanelet::Origin({1.316013710965, 103.79549063793})},
      {"singapore-queenstown.osm", lanelet::Origin({1.29779876, 103.7826437429})}};

  std::map<std::string, lanelet::Origin> result;
  for (const auto &[filename, origin] : map_origins)
  {
    result[maps_directory_ + filename] = origin;
  }
  return result;
}