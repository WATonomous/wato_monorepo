#pragma once

#include <cmath>

namespace eidos {

struct UtmCoordinate {
  double easting;
  double northing;
  double altitude;
  int zone;
  bool is_north;
};

/**
 * @brief Convert WGS84 lat/lon/alt to UTM coordinates.
 *
 * Standard Transverse Mercator projection using WGS84 ellipsoid constants.
 * Zone is auto-determined from longitude.
 */
inline UtmCoordinate latLonToUtm(double lat_deg, double lon_deg, double alt) {
  // WGS84 ellipsoid constants
  constexpr double a = 6378137.0;              // semi-major axis (m)
  constexpr double f = 1.0 / 298.257223563;    // flattening
  constexpr double e2 = 2.0 * f - f * f;       // first eccentricity squared
  constexpr double e_prime2 = e2 / (1.0 - e2); // second eccentricity squared
  constexpr double k0 = 0.9996;                // scale factor

  constexpr double deg2rad = M_PI / 180.0;

  double lat_rad = lat_deg * deg2rad;
  double lon_rad = lon_deg * deg2rad;

  int zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
  double lon_origin = (zone - 1) * 6.0 - 180.0 + 3.0;  // center of zone
  double lon_origin_rad = lon_origin * deg2rad;

  double sin_lat = std::sin(lat_rad);
  double cos_lat = std::cos(lat_rad);
  double tan_lat = std::tan(lat_rad);

  double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  double T = tan_lat * tan_lat;
  double C = e_prime2 * cos_lat * cos_lat;
  double A = cos_lat * (lon_rad - lon_origin_rad);

  double M = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * lat_rad
               - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * lat_rad)
               + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat_rad)
               - (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat_rad));

  double A2 = A * A;
  double A3 = A2 * A;
  double A4 = A3 * A;
  double A5 = A4 * A;
  double A6 = A5 * A;

  double easting = k0 * N * (A + (1.0 - T + C) * A3 / 6.0
                   + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e_prime2) * A5 / 120.0)
                   + 500000.0;

  double northing = k0 * (M + N * tan_lat * (A2 / 2.0
                    + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0
                    + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e_prime2) * A6 / 720.0));

  bool is_north = lat_deg >= 0.0;
  if (!is_north) {
    northing += 10000000.0;  // southern hemisphere offset
  }

  return {easting, northing, alt, zone, is_north};
}

}  // namespace eidos
