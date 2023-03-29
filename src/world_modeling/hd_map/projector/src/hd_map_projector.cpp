#include "hd_map_projector.hpp"

namespace world_modeling::hd_map
{
  // Constructor implementation
  WGS84ToLocalProjector::WGS84ToLocalProjector(double lat, double lon)
      : utm_projector_(lanelet::Origin(lat, lon))  // Initialize the UTMProjector with the provided origin
  {
    lanelet::GPSPoint origin_gps(lat, lon, 0);     // Create a GPS point representing the origin
    origin_ = utm_projector_.forward(origin_gps);  // Convert the origin GPS point to a local metric point
  }

  // forward() method implementation
  lanelet::BasicPoint3d WGS84ToLocalProjector::forward(const lanelet::GPSPoint& gps_point) const
  {
    // Project the GPS point to the local metric coordinate system using
    // the UTMProjector, then subtract the origin point to get the final
    // local metric coordinates.
    return utm_projector_.forward(gps_point) - origin_;
  }

  // reverse() method implementation
  lanelet::GPSPoint WGS84ToLocalProjector::reverse(const lanelet::BasicPoint3d& local_point) const
  {
    // Add the origin point to the local metric point, then use the
    // UTMProjector to convert it back to GPS coordinates.
    return utm_projector_.reverse(local_point + origin_);
  }
}  // namespace world_modeling::hd_map
