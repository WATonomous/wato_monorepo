#include "hd_map_projector.hpp"

namespace world_modeling::hd_map
{
  // Constructor implementation
  WGS84ToLocalProjector::WGS84ToLocalProjector(double lat, double lon)
  {
    lanelet::GPSPoint origin_gps;  // Create a GPS point representing the origin
    origin_gps.lat = lat;
    origin_gps.lon = lon;
    origin_gps.ele = 0;

    utm_projector_ = lanelet::projection::UtmProjector(lanelet::Origin(origin_gps), true, false);  // Initialize the UtmProjector with the provided origin
    origin_ = utm_projector_.forward(origin_gps);  // Convert the origin GPS point to a local metric point
  }

  // forward() method implementation
  lanelet::BasicPoint3d WGS84ToLocalProjector::forward(const lanelet::GPSPoint& gps_point) const
  {
    // Project the GPS point to the local metric coordinate system using
    // the UtmProjector, then subtract the origin point to get the final
    // local metric coordinates.
    return utm_projector_.forward(gps_point) - origin_;
  }

  // reverse() method implementation
  lanelet::GPSPoint WGS84ToLocalProjector::reverse(const lanelet::BasicPoint3d& local_point) const
  {
    // Add the origin point to the local metric point, then use the
    // UtmProjector to convert it back to GPS coordinates.
    return utm_projector_.reverse(local_point + origin_);
  }
}  // namespace world_modeling::hd_map
