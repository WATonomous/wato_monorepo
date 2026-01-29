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

#include <cmath>
#include <limits>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prediction/map_interface_core.hpp"

using prediction::LaneletInfo;
using prediction::MapInterfaceCore;

namespace
{

// Helper to create a point
geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Helper to create a simple lanelet
lanelet_msgs::msg::Lanelet makeLanelet(int64_t id, double x_start, double x_end, double y = 0.0)
{
  lanelet_msgs::msg::Lanelet lanelet;
  lanelet.id = id;
  lanelet.centerline.push_back(makePoint(x_start, y));
  lanelet.centerline.push_back(makePoint(x_end, y));
  lanelet.speed_limit_mps = 10.0;
  lanelet.lanelet_type = "road";
  lanelet.can_change_left = false;
  lanelet.can_change_right = false;
  lanelet.left_lane_id = -1;
  lanelet.right_lane_id = -1;
  return lanelet;
}

}  // namespace

TEST_CASE("MapInterfaceCore distance calculations", "[map_interface_core]")
{
  SECTION("distancePointToSegment - point on segment")
  {
    auto p = makePoint(5.0, 0.0);
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(10.0, 0.0);

    double dist = MapInterfaceCore::distancePointToSegment(p, a, b);
    REQUIRE(dist == Catch::Approx(0.0));
  }

  SECTION("distancePointToSegment - point perpendicular to segment")
  {
    auto p = makePoint(5.0, 3.0);
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(10.0, 0.0);

    double dist = MapInterfaceCore::distancePointToSegment(p, a, b);
    REQUIRE(dist == Catch::Approx(3.0));
  }

  SECTION("distancePointToSegment - point beyond segment end")
  {
    auto p = makePoint(15.0, 0.0);
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(10.0, 0.0);

    double dist = MapInterfaceCore::distancePointToSegment(p, a, b);
    REQUIRE(dist == Catch::Approx(5.0));
  }

  SECTION("distancePointToSegment - point before segment start")
  {
    auto p = makePoint(-5.0, 0.0);
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(10.0, 0.0);

    double dist = MapInterfaceCore::distancePointToSegment(p, a, b);
    REQUIRE(dist == Catch::Approx(5.0));
  }

  SECTION("distancePointToSegment - zero length segment")
  {
    auto p = makePoint(3.0, 4.0);
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(0.0, 0.0);

    double dist = MapInterfaceCore::distancePointToSegment(p, a, b);
    REQUIRE(dist == Catch::Approx(5.0));  // 3-4-5 triangle
  }

  SECTION("distanceToLanelet - empty centerline")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 1;
    // No centerline points

    auto p = makePoint(0.0, 0.0);
    double dist = MapInterfaceCore::distanceToLanelet(p, lanelet);
    REQUIRE(dist == std::numeric_limits<double>::max());
  }

  SECTION("distanceToLanelet - single point centerline")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 1;
    lanelet.centerline.push_back(makePoint(3.0, 4.0));

    auto p = makePoint(0.0, 0.0);
    double dist = MapInterfaceCore::distanceToLanelet(p, lanelet);
    REQUIRE(dist == Catch::Approx(5.0));  // 3-4-5 triangle
  }

  SECTION("distanceToLanelet - multi-segment centerline")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 1;
    lanelet.centerline.push_back(makePoint(0.0, 0.0));
    lanelet.centerline.push_back(makePoint(10.0, 0.0));
    lanelet.centerline.push_back(makePoint(10.0, 10.0));

    // Point closest to first segment
    auto p1 = makePoint(5.0, 2.0);
    double dist1 = MapInterfaceCore::distanceToLanelet(p1, lanelet);
    REQUIRE(dist1 == Catch::Approx(2.0));

    // Point closest to second segment
    auto p2 = makePoint(12.0, 5.0);
    double dist2 = MapInterfaceCore::distanceToLanelet(p2, lanelet);
    REQUIRE(dist2 == Catch::Approx(2.0));
  }
}

TEST_CASE("MapInterfaceCore caching", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Initial state is empty")
  {
    REQUIRE(core.isCacheEmpty());
    REQUIRE(core.getCacheSize() == 0);
  }

  SECTION("Cache single lanelet")
  {
    auto lanelet = makeLanelet(1, 0.0, 10.0);
    core.cacheLanelet(lanelet);

    REQUIRE_FALSE(core.isCacheEmpty());
    REQUIRE(core.getCacheSize() == 1);
  }

  SECTION("Cache multiple lanelets")
  {
    std::vector<lanelet_msgs::msg::Lanelet> lanelets;
    lanelets.push_back(makeLanelet(1, 0.0, 10.0));
    lanelets.push_back(makeLanelet(2, 10.0, 20.0));
    lanelets.push_back(makeLanelet(3, 20.0, 30.0));

    core.cacheLanelets(lanelets);

    REQUIRE(core.getCacheSize() == 3);
  }

  SECTION("Update existing lanelet")
  {
    auto lanelet1 = makeLanelet(1, 0.0, 10.0);
    lanelet1.speed_limit_mps = 10.0;
    core.cacheLanelet(lanelet1);

    // Update with new speed limit
    auto lanelet2 = makeLanelet(1, 0.0, 10.0);
    lanelet2.speed_limit_mps = 20.0;
    core.cacheLanelet(lanelet2);

    REQUIRE(core.getCacheSize() == 1);
    auto speed = core.getSpeedLimit(1);
    REQUIRE(speed.has_value());
    REQUIRE(speed.value() == Catch::Approx(20.0));
  }

  SECTION("Clear cache")
  {
    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    REQUIRE(core.getCacheSize() == 2);

    core.clearCache();
    REQUIRE(core.isCacheEmpty());
    REQUIRE(core.getCacheSize() == 0);
  }
}

TEST_CASE("MapInterfaceCore findNearestLanelet", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Empty cache returns nullopt")
  {
    auto result = core.findNearestLanelet(makePoint(0.0, 0.0));
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Find nearest from single lanelet")
  {
    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));

    auto result = core.findNearestLanelet(makePoint(5.0, 2.0));
    REQUIRE(result.has_value());
    REQUIRE(result.value() == 1);
  }

  SECTION("Find nearest from multiple lanelets")
  {
    // Lanelet 1: y = 0
    core.cacheLanelet(makeLanelet(1, 0.0, 10.0, 0.0));
    // Lanelet 2: y = 10
    core.cacheLanelet(makeLanelet(2, 0.0, 10.0, 10.0));
    // Lanelet 3: y = 20
    core.cacheLanelet(makeLanelet(3, 0.0, 10.0, 20.0));

    // Point closest to lanelet 2
    auto result = core.findNearestLanelet(makePoint(5.0, 12.0));
    REQUIRE(result.has_value());
    REQUIRE(result.value() == 2);
  }
}

TEST_CASE("MapInterfaceCore getLaneletById", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Non-existent lanelet returns nullopt")
  {
    auto result = core.getLaneletById(999);
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Get existing lanelet")
  {
    auto lanelet = makeLanelet(42, 0.0, 10.0);
    lanelet.speed_limit_mps = 15.5;
    lanelet.successor_ids = {43, 44};
    core.cacheLanelet(lanelet);

    auto result = core.getLaneletById(42);
    REQUIRE(result.has_value());
    REQUIRE(result->id == 42);
    REQUIRE(result->speed_limit == Catch::Approx(15.5));
    REQUIRE(result->following_lanelets.size() == 2);
    REQUIRE(result->following_lanelets[0] == 43);
    REQUIRE(result->following_lanelets[1] == 44);
  }
}

TEST_CASE("MapInterfaceCore getSpeedLimit", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Non-existent lanelet returns nullopt")
  {
    auto result = core.getSpeedLimit(999);
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Get speed limit")
  {
    auto lanelet = makeLanelet(1, 0.0, 10.0);
    lanelet.speed_limit_mps = 25.0;
    core.cacheLanelet(lanelet);

    auto result = core.getSpeedLimit(1);
    REQUIRE(result.has_value());
    REQUIRE(result.value() == Catch::Approx(25.0));
  }
}

TEST_CASE("MapInterfaceCore getPossibleFutureLanelets", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Non-existent lanelet returns empty vector")
  {
    auto result = core.getPossibleFutureLanelets(999);
    // Should still contain the starting ID even if not in cache
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 999);
  }

  SECTION("Linear chain of lanelets")
  {
    // Create chain: 1 -> 2 -> 3 -> 4
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.successor_ids = {2};
    auto l2 = makeLanelet(2, 10.0, 20.0);
    l2.successor_ids = {3};
    auto l3 = makeLanelet(3, 20.0, 30.0);
    l3.successor_ids = {4};
    auto l4 = makeLanelet(4, 30.0, 40.0);

    core.cacheLanelets({l1, l2, l3, l4});

    // Depth 1: should get 1, 2
    auto result1 = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result1.size() == 2);

    // Depth 3: should get 1, 2, 3, 4
    auto result3 = core.getPossibleFutureLanelets(1, 3);
    REQUIRE(result3.size() == 4);
  }

  SECTION("Branching lanelets")
  {
    // Create branch: 1 -> {2, 3}
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.successor_ids = {2, 3};
    auto l2 = makeLanelet(2, 10.0, 20.0, 5.0);
    auto l3 = makeLanelet(3, 10.0, 20.0, -5.0);

    core.cacheLanelets({l1, l2, l3});

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 3);
  }

  SECTION("Lane change options")
  {
    // Create: 1 with left lane 2
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.can_change_left = true;
    l1.left_lane_id = 2;
    auto l2 = makeLanelet(2, 0.0, 10.0, 3.5);

    core.cacheLanelets({l1, l2});

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 2);
  }
}

TEST_CASE("MapInterfaceCore isCrosswalkNearby", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("No crosswalks returns false")
  {
    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));

    auto result = core.isCrosswalkNearby(makePoint(5.0, 0.0), 5.0);
    REQUIRE_FALSE(result);
  }

  SECTION("Crosswalk within radius returns true")
  {
    auto crosswalk = makeLanelet(1, 0.0, 5.0);
    crosswalk.lanelet_type = "crosswalk";
    core.cacheLanelet(crosswalk);

    auto result = core.isCrosswalkNearby(makePoint(2.5, 2.0), 5.0);
    REQUIRE(result);
  }

  SECTION("Crosswalk outside radius returns false")
  {
    auto crosswalk = makeLanelet(1, 0.0, 5.0);
    crosswalk.lanelet_type = "crosswalk";
    core.cacheLanelet(crosswalk);

    auto result = core.isCrosswalkNearby(makePoint(2.5, 10.0), 5.0);
    REQUIRE_FALSE(result);
  }
}

TEST_CASE("MapInterfaceCore laneletMsgToInfo", "[map_interface_core]")
{
  SECTION("Full lanelet conversion")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 123;
    lanelet.centerline.push_back(makePoint(0.0, 0.0));
    lanelet.centerline.push_back(makePoint(10.0, 0.0));
    lanelet.speed_limit_mps = 13.89;  // 50 km/h
    lanelet.successor_ids = {124, 125};

    auto info = MapInterfaceCore::laneletMsgToInfo(lanelet);

    REQUIRE(info.id == 123);
    REQUIRE(info.centerline.size() == 2);
    REQUIRE(info.speed_limit == Catch::Approx(13.89));
    REQUIRE(info.following_lanelets.size() == 2);
    REQUIRE(info.following_lanelets[0] == 124);
    REQUIRE(info.following_lanelets[1] == 125);
  }

  SECTION("Empty centerline and no successors")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 1;
    lanelet.speed_limit_mps = 0.0;

    auto info = MapInterfaceCore::laneletMsgToInfo(lanelet);

    REQUIRE(info.id == 1);
    REQUIRE(info.centerline.empty());
    REQUIRE(info.speed_limit == Catch::Approx(0.0));
    REQUIRE(info.following_lanelets.empty());
  }
}

TEST_CASE("MapInterfaceCore LRU eviction", "[map_interface_core]")
{
  SECTION("Evicts least recently used when cache is full")
  {
    MapInterfaceCore core(3);  // Small cache of 3

    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    core.cacheLanelet(makeLanelet(3, 20.0, 30.0));
    REQUIRE(core.getCacheSize() == 3);

    // Add a 4th lanelet, should evict lanelet 1 (oldest)
    core.cacheLanelet(makeLanelet(4, 30.0, 40.0));

    REQUIRE(core.getCacheSize() == 3);
    REQUIRE_FALSE(core.getLaneletById(1).has_value());  // Evicted
    REQUIRE(core.getLaneletById(2).has_value());
    REQUIRE(core.getLaneletById(3).has_value());
    REQUIRE(core.getLaneletById(4).has_value());
  }

  SECTION("Accessing lanelet updates LRU order")
  {
    MapInterfaceCore core(3);

    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    core.cacheLanelet(makeLanelet(3, 20.0, 30.0));

    // Access lanelet 1, making it most recently used
    core.getLaneletById(1);

    // Add lanelet 4, should now evict lanelet 2 (oldest)
    core.cacheLanelet(makeLanelet(4, 30.0, 40.0));

    REQUIRE(core.getLaneletById(1).has_value());  // Still present
    REQUIRE_FALSE(core.getLaneletById(2).has_value());  // Evicted
    REQUIRE(core.getLaneletById(3).has_value());
    REQUIRE(core.getLaneletById(4).has_value());
  }

  SECTION("getSpeedLimit updates LRU order")
  {
    MapInterfaceCore core(3);

    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    core.cacheLanelet(makeLanelet(3, 20.0, 30.0));

    // Access lanelet 1 via getSpeedLimit
    core.getSpeedLimit(1);

    // Add lanelet 4
    core.cacheLanelet(makeLanelet(4, 30.0, 40.0));

    REQUIRE(core.getLaneletById(1).has_value());  // Still present
    REQUIRE_FALSE(core.getLaneletById(2).has_value());  // Evicted
  }

  SECTION("Re-caching existing lanelet updates LRU order")
  {
    MapInterfaceCore core(3);

    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    core.cacheLanelet(makeLanelet(3, 20.0, 30.0));

    // Re-cache lanelet 1 with updated data
    auto updated = makeLanelet(1, 0.0, 15.0);
    updated.speed_limit_mps = 99.0;
    core.cacheLanelet(updated);

    // Add lanelet 4, should evict lanelet 2
    core.cacheLanelet(makeLanelet(4, 30.0, 40.0));

    REQUIRE(core.getLaneletById(1).has_value());
    REQUIRE(core.getSpeedLimit(1).value() == Catch::Approx(99.0));
    REQUIRE_FALSE(core.getLaneletById(2).has_value());
  }
}

TEST_CASE("MapInterfaceCore custom cache size", "[map_interface_core]")
{
  SECTION("Default cache size")
  {
    MapInterfaceCore core;
    REQUIRE(core.getMaxCacheSize() == MapInterfaceCore::DEFAULT_MAX_CACHE_SIZE);
  }

  SECTION("Custom cache size")
  {
    MapInterfaceCore core(500);
    REQUIRE(core.getMaxCacheSize() == 500);
  }

  SECTION("Cache size of 1")
  {
    MapInterfaceCore core(1);

    core.cacheLanelet(makeLanelet(1, 0.0, 10.0));
    REQUIRE(core.getCacheSize() == 1);

    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    REQUIRE(core.getCacheSize() == 1);
    REQUIRE_FALSE(core.getLaneletById(1).has_value());
    REQUIRE(core.getLaneletById(2).has_value());
  }
}

TEST_CASE("MapInterfaceCore diagonal segment distance", "[map_interface_core]")
{
  SECTION("Point perpendicular to 45-degree segment")
  {
    // Segment from (0,0) to (10,10), point at (5,5) + perpendicular offset
    auto a = makePoint(0.0, 0.0);
    auto b = makePoint(10.0, 10.0);

    // Point on the line
    auto p_on_line = makePoint(5.0, 5.0);
    REQUIRE(MapInterfaceCore::distancePointToSegment(p_on_line, a, b) == Catch::Approx(0.0).margin(1e-9));

    // Point perpendicular to midpoint (offset by sqrt(2) perpendicular)
    auto p_offset = makePoint(6.0, 4.0);  // sqrt(2) away perpendicular
    double expected_dist = std::sqrt(2.0);
    REQUIRE(MapInterfaceCore::distancePointToSegment(p_offset, a, b) == Catch::Approx(expected_dist).margin(1e-9));
  }

  SECTION("Vertical segment")
  {
    auto a = makePoint(5.0, 0.0);
    auto b = makePoint(5.0, 10.0);

    auto p = makePoint(8.0, 5.0);
    REQUIRE(MapInterfaceCore::distancePointToSegment(p, a, b) == Catch::Approx(3.0));
  }
}

TEST_CASE("MapInterfaceCore getPossibleFutureLanelets advanced", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Depth 0 returns only starting lanelet")
  {
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.successor_ids = {2, 3};
    core.cacheLanelet(l1);
    core.cacheLanelet(makeLanelet(2, 10.0, 20.0));
    core.cacheLanelet(makeLanelet(3, 10.0, 20.0));

    auto result = core.getPossibleFutureLanelets(1, 0);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);
  }

  SECTION("Right lane change option")
  {
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.can_change_right = true;
    l1.right_lane_id = 2;
    auto l2 = makeLanelet(2, 0.0, 10.0, -3.5);

    core.cacheLanelets({l1, l2});

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 2);
  }

  SECTION("Both left and right lane change options")
  {
    auto l1 = makeLanelet(1, 0.0, 10.0, 0.0);
    l1.can_change_left = true;
    l1.left_lane_id = 2;
    l1.can_change_right = true;
    l1.right_lane_id = 3;

    auto l2 = makeLanelet(2, 0.0, 10.0, 3.5);
    auto l3 = makeLanelet(3, 0.0, 10.0, -3.5);

    core.cacheLanelets({l1, l2, l3});

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 3);
  }

  SECTION("Handles cycles in lanelet graph")
  {
    // Create a cycle: 1 -> 2 -> 3 -> 1
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.successor_ids = {2};
    auto l2 = makeLanelet(2, 10.0, 20.0);
    l2.successor_ids = {3};
    auto l3 = makeLanelet(3, 20.0, 30.0);
    l3.successor_ids = {1};  // Back to 1

    core.cacheLanelets({l1, l2, l3});

    // Should not infinite loop, each lanelet visited once
    auto result = core.getPossibleFutureLanelets(1, 10);
    REQUIRE(result.size() == 3);
  }

  SECTION("Lane change with invalid ID (-1) is ignored")
  {
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.can_change_left = true;
    l1.left_lane_id = -1;  // Invalid

    core.cacheLanelet(l1);

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);
  }

  SECTION("Lane change flag false with valid ID is not followed")
  {
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.can_change_left = false;
    l1.left_lane_id = 2;

    auto l2 = makeLanelet(2, 0.0, 10.0, 3.5);

    core.cacheLanelets({l1, l2});

    auto result = core.getPossibleFutureLanelets(1, 1);
    REQUIRE(result.size() == 1);  // Only lanelet 1
  }

  SECTION("Complex multi-path scenario")
  {
    // Create: 1 -> {2, 3}, 2 -> 4, 3 -> 4 (diamond pattern)
    auto l1 = makeLanelet(1, 0.0, 10.0);
    l1.successor_ids = {2, 3};

    auto l2 = makeLanelet(2, 10.0, 20.0, 3.0);
    l2.successor_ids = {4};

    auto l3 = makeLanelet(3, 10.0, 20.0, -3.0);
    l3.successor_ids = {4};

    auto l4 = makeLanelet(4, 20.0, 30.0);

    core.cacheLanelets({l1, l2, l3, l4});

    auto result = core.getPossibleFutureLanelets(1, 2);
    REQUIRE(result.size() == 4);  // All 4 lanelets, 4 visited once
  }
}

TEST_CASE("MapInterfaceCore findNearestLanelet edge cases", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("All lanelets have empty centerlines returns nullopt")
  {
    lanelet_msgs::msg::Lanelet l1;
    l1.id = 1;
    lanelet_msgs::msg::Lanelet l2;
    l2.id = 2;

    core.cacheLanelets({l1, l2});

    // All return max distance, so no lanelet is "nearest" - returns nullopt
    auto result = core.findNearestLanelet(makePoint(0.0, 0.0));
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Point equidistant from two lanelets")
  {
    // Two parallel lanelets at y = 5 and y = -5
    core.cacheLanelet(makeLanelet(1, 0.0, 10.0, 5.0));
    core.cacheLanelet(makeLanelet(2, 0.0, 10.0, -5.0));

    // Point at origin is equidistant
    auto result = core.findNearestLanelet(makePoint(5.0, 0.0));
    REQUIRE(result.has_value());
    // One of the two should be returned (implementation-dependent)
    REQUIRE((result.value() == 1 || result.value() == 2));
  }

  SECTION("Negative coordinates")
  {
    core.cacheLanelet(makeLanelet(1, -20.0, -10.0, -5.0));

    auto result = core.findNearestLanelet(makePoint(-15.0, -3.0));
    REQUIRE(result.has_value());
    REQUIRE(result.value() == 1);
  }
}

TEST_CASE("MapInterfaceCore isCrosswalkNearby edge cases", "[map_interface_core]")
{
  MapInterfaceCore core;

  SECTION("Mixed lanelet types - only detects crosswalks")
  {
    auto road = makeLanelet(1, 0.0, 10.0, 0.0);
    road.lanelet_type = "road";

    auto crosswalk = makeLanelet(2, 5.0, 15.0, 10.0);
    crosswalk.lanelet_type = "crosswalk";

    core.cacheLanelets({road, crosswalk});

    // Point near road but far from crosswalk
    REQUIRE_FALSE(core.isCrosswalkNearby(makePoint(5.0, 1.0), 5.0));

    // Point near crosswalk
    REQUIRE(core.isCrosswalkNearby(makePoint(10.0, 8.0), 5.0));
  }

  SECTION("Multiple crosswalks - returns true if any nearby")
  {
    auto cw1 = makeLanelet(1, 0.0, 5.0, 0.0);
    cw1.lanelet_type = "crosswalk";
    auto cw2 = makeLanelet(2, 100.0, 105.0, 0.0);
    cw2.lanelet_type = "crosswalk";

    core.cacheLanelets({cw1, cw2});

    REQUIRE(core.isCrosswalkNearby(makePoint(2.5, 1.0), 5.0));
    REQUIRE(core.isCrosswalkNearby(makePoint(102.5, 1.0), 5.0));
    REQUIRE_FALSE(core.isCrosswalkNearby(makePoint(50.0, 0.0), 5.0));
  }

  SECTION("Radius of 0 - only exact matches")
  {
    auto crosswalk = makeLanelet(1, 0.0, 10.0);
    crosswalk.lanelet_type = "crosswalk";
    core.cacheLanelet(crosswalk);

    // Point exactly on centerline
    REQUIRE(core.isCrosswalkNearby(makePoint(5.0, 0.0), 0.0));

    // Point slightly off
    REQUIRE_FALSE(core.isCrosswalkNearby(makePoint(5.0, 0.001), 0.0));
  }

  SECTION("Empty cache returns false")
  {
    REQUIRE_FALSE(core.isCrosswalkNearby(makePoint(0.0, 0.0), 100.0));
  }
}

TEST_CASE("MapInterfaceCore distanceToLanelet with 3D points", "[map_interface_core]")
{
  // The implementation only uses x,y - verify z is ignored
  SECTION("Z coordinate is ignored in distance calculation")
  {
    lanelet_msgs::msg::Lanelet lanelet;
    lanelet.id = 1;
    lanelet.centerline.push_back(makePoint(0.0, 0.0, 0.0));
    lanelet.centerline.push_back(makePoint(10.0, 0.0, 0.0));

    // Point at z=100 should have same 2D distance as z=0
    auto p_z0 = makePoint(5.0, 3.0, 0.0);
    auto p_z100 = makePoint(5.0, 3.0, 100.0);

    double dist_z0 = MapInterfaceCore::distanceToLanelet(p_z0, lanelet);
    double dist_z100 = MapInterfaceCore::distanceToLanelet(p_z100, lanelet);

    REQUIRE(dist_z0 == Catch::Approx(dist_z100));
    REQUIRE(dist_z0 == Catch::Approx(3.0));
  }
}
