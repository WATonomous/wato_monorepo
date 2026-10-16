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

#include <lanelet2_projection/LocalCartesian.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "world_model/map_projector.hpp"

namespace
{
world_model::MapConfig ovalConfig()
{
  world_model::MapConfig config;
  config.origin_lat = 45.5017;
  config.origin_lon = -73.5673;
  return config;
}

void checkRoundTrip(const world_model::MapProjector & projector, const lanelet::GPSPoint & point)
{
  const auto round_trip = projector.reverse(projector.forward(point));
  CHECK(std::abs(round_trip.lat - point.lat) < 1e-9);
  CHECK(std::abs(round_trip.lon - point.lon) < 1e-9);
  CHECK(std::abs(round_trip.ele - point.ele) < 1e-6);
}
}  // namespace

TEST_CASE("Fixed UTM matches the centred oval OpenDRIVE coordinates", "[map_projection]")
{
  auto config = ovalConfig();
  config.origin_alt = 298.0;
  const world_model::MapProjector projector(config);
  const auto centre = projector.forward({45.5017, -73.5673, 298.0});
  CHECK(centre.norm() < 1e-6);

  // OSM node 45 is the first reference-line point of road 0 in oval.xodr.
  const lanelet::GPSPoint point{45.502140622, -73.567840803, 300.0};
  const auto projected = projector.forward(point);
  CHECK(std::abs(projected.x() - (-43.12266144066391)) < 1e-6);
  CHECK(std::abs(projected.y() - 48.198029877475456) < 1e-6);
  CHECK(std::abs(projected.z() - 2.0) < 1e-6);
  checkRoundTrip(projector, point);
}

TEST_CASE("Local Cartesian honours geographic origin and Town10HD convention", "[map_projection]")
{
  auto config = ovalConfig();
  config.projector_type = "local_cartesian";
  config.origin_alt = 298.0;
  const world_model::MapProjector geographic(config);
  CHECK(geographic.forward({45.5017, -73.5673, 298.0}).norm() < 1e-6);
  checkRoundTrip(geographic, {45.502140622, -73.567840803, 300.0});

  config.origin_lat = 0.0;
  config.origin_lon = 0.0;
  config.origin_alt = 0.0;
  const world_model::MapProjector town(config);
  const lanelet::projection::LocalCartesianProjector original(lanelet::Origin({0.0, 0.0, 0.0}));
  const lanelet::BasicPoint3d carla_point{125.0, -30.0, 2.0};
  CHECK((town.forward(original.reverse(carla_point)) - carla_point).norm() < 1e-6);
}

TEST_CASE("TF alignment rotates absolute UTM into the Eidos map", "[map_projection]")
{
  auto config = ovalConfig();
  config.alignment_mode = "tf";
  config.origin_alt = 298.0;  // Must not also subtract this in TF mode.
  geometry_msgs::msg::Transform transform;
  const double half_pi = std::acos(-1.0) / 2.0;
  transform.rotation.z = std::sin(half_pi / 2.0);
  transform.rotation.w = std::cos(half_pi / 2.0);
  // Rotate 90 degrees and place the geographic origin at map (10, 20, 30).
  transform.translation.x = 5039683.63022774272 + 10.0;
  transform.translation.y = -611928.71216549247 + 20.0;
  transform.translation.z = -298.0 + 30.0;
  const world_model::MapProjector projector(config, transform);
  CHECK((projector.forward({45.5017, -73.5673, 298.0}) - lanelet::BasicPoint3d(10, 20, 30)).norm() < 1e-6);

  const lanelet::GPSPoint point{45.502140622, -73.567840803, 300.0};
  const lanelet::BasicPoint3d expected{10.0 - 48.198029877475456, 20.0 - 43.12266144066391, 32.0};
  CHECK((projector.forward(point) - expected).norm() < 1e-6);
  checkRoundTrip(projector, point);
}

TEST_CASE("TF alignment handles roll and a local ENU source frame", "[map_projection]")
{
  auto config = ovalConfig();
  config.projector_type = "local_cartesian";
  config.alignment_mode = "tf";
  config.projected_frame = "enu";
  config.origin_alt = 298.0;
  geometry_msgs::msg::Transform transform;
  const double half_pi = std::acos(-1.0) / 2.0;
  transform.rotation.x = std::sin(half_pi / 2.0);
  transform.rotation.w = std::cos(half_pi / 2.0);
  transform.translation.x = 10.0;
  transform.translation.y = 20.0;
  transform.translation.z = 30.0;
  const lanelet::projection::LocalCartesianProjector source(
    lanelet::Origin({config.origin_lat, config.origin_lon, config.origin_alt}));
  const auto point = source.reverse({5.0, 7.0, 11.0});
  const world_model::MapProjector projector(config, transform);
  CHECK((projector.forward(point) - lanelet::BasicPoint3d(15, 9, 37)).norm() < 1e-6);
  checkRoundTrip(projector, point);
}

TEST_CASE("Map configuration rejects ambiguous modes and invalid transforms", "[map_projection]")
{
  auto config = ovalConfig();
  SECTION("Unknown projector")
  {
    config.projector_type = "carla";
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
  }
  SECTION("Unknown alignment")
  {
    config.alignment_mode = "wait";
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
  }
  SECTION("Invalid latitude")
  {
    config.origin_lat = 91.0;
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
  }
  SECTION("Nonfinite longitude")
  {
    config.origin_lon = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
  }
  SECTION("TF mode requires a transform and source frame")
  {
    config.alignment_mode = "tf";
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
    config.projected_frame.clear();
    CHECK_THROWS_AS(world_model::MapProjector(config), std::invalid_argument);
  }
  SECTION("Fixed mode rejects a supplied transform")
  {
    CHECK_THROWS_AS(world_model::MapProjector(config, geometry_msgs::msg::Transform()), std::invalid_argument);
  }
  SECTION("TF mode rejects zero quaternion")
  {
    config.alignment_mode = "tf";
    geometry_msgs::msg::Transform transform;
    transform.rotation.w = 0.0;
    CHECK_THROWS_AS(world_model::MapProjector(config, transform), std::invalid_argument);
  }
  SECTION("TF mode rejects nonfinite translation")
  {
    config.alignment_mode = "tf";
    geometry_msgs::msg::Transform transform;
    transform.translation.x = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(world_model::MapProjector(config, transform), std::invalid_argument);
  }
}
