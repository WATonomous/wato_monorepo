#ifndef HD_MAP_PROJECTOR_HPP_
#define HD_MAP_PROJECTOR_HPP_

#include <vector>

// Include the necessary Lanelet2 headers
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_projection/UTM.h>

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"

namespace world_modeling::hd_map
{
  // Define a new class WGS84ToLocalProjector to handle conversions
  // between WGS84 (lat/lon) and a local metric coordinate system.
  class WGS84ToLocalProjector
  {
  public:
    // Constructor takes the latitude and longitude of the origin
    // point as arguments.
    WGS84ToLocalProjector(double lat, double lon);

    // forward() method takes a GPS point and returns its corresponding
    // local metric coordinates.
    lanelet::BasicPoint3d forward(const lanelet::GPSPoint& gps_point) const;

    // reverse() method takes a local metric point and returns its
    // corresponding GPS coordinates.
    lanelet::GPSPoint reverse(const lanelet::BasicPoint3d& local_point) const;

  private:
    // An instance of the UtmProjector class from Lanelet2.
    lanelet::projection::UtmProjector::Ptr utm_projector_;

    // The origin point in the local metric coordinate system.
    lanelet::BasicPoint3d origin_;
  };
}  // namespace world_modeling::hd_map

#endif  // HD_MAP_PROJECTOR_HPP_
