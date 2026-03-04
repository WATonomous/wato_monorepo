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

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/msg/lanelet.hpp"
#include "lanelet_msgs/msg/lanelet_ahead.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"
#include "lanelet_msgs/srv/set_route.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model/types/entity_3d.hpp"
#include "world_model/types/entity_buffer.hpp"
#include "world_model/world_state.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

// Entity Buffer Tests

TEST_CASE("EntityBuffer basic operations", "[entity_buffer]")
{
  world_model::EntityBuffer<world_model::Car> buffer;

  SECTION("Buffer starts empty")
  {
    REQUIRE(buffer.size() == 0);
    REQUIRE(buffer.getAll().empty());
  }

  SECTION("Upsert creates new entity")
  {
    world_model::Car default_car;
    buffer.upsert("42", default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 10.0;
      det.bbox.center.position.y = 20.0;
      car.history.push_front(det);
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get("42");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->id() == "42");
    REQUIRE(retrieved->pose().position.x == Catch::Approx(10.0));
  }

  SECTION("Upsert updates existing entity")
  {
    world_model::Car default_car;

    // First insert
    buffer.upsert("42", default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 10.0;
      car.history.push_front(det);
    });

    // Update
    buffer.upsert("42", default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 30.0;
      car.history.push_front(det);
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get("42");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->pose().position.x == Catch::Approx(30.0));
    REQUIRE(retrieved->history.size() == 2);
  }

  SECTION("Get returns nullopt for non-existent entity")
  {
    auto retrieved = buffer.get("999");
    REQUIRE_FALSE(retrieved.has_value());
  }

  SECTION("Prune removes entities matching predicate")
  {
    world_model::Car default_car;

    for (int i = 0; i < 5; ++i) {
      std::string sid = std::to_string(i);
      buffer.upsert(sid, default_car, [sid, i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = sid;
        det.bbox.center.position.x = static_cast<double>(i * 10);
        car.history.push_front(det);
      });
    }

    REQUIRE(buffer.size() == 5);

    // Prune entities with x < 25 (removes 0, 1, 2)
    buffer.prune([](const world_model::Car & car) { return car.pose().position.x < 25.0; });

    REQUIRE(buffer.size() == 2);
    REQUIRE_FALSE(buffer.get("0").has_value());
    REQUIRE_FALSE(buffer.get("1").has_value());
    REQUIRE_FALSE(buffer.get("2").has_value());
    REQUIRE(buffer.get("3").has_value());
    REQUIRE(buffer.get("4").has_value());
  }
}

// WorldState Tests

TEST_CASE("WorldState entity type access", "[world_state]")
{
  world_model::WorldState state;

  SECTION("Can access all entity buffer types")
  {
    REQUIRE(state.buffer<world_model::Car>().size() == 0);
    REQUIRE(state.buffer<world_model::Human>().size() == 0);
    REQUIRE(state.buffer<world_model::Bicycle>().size() == 0);
    REQUIRE(state.buffer<world_model::Motorcycle>().size() == 0);
    REQUIRE(state.buffer<world_model::TrafficLight>().size() == 0);
  }

  SECTION("Buffers are independent")
  {
    world_model::Car default_car;
    world_model::Human default_human;

    state.buffer<world_model::Car>().upsert("1", default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "1";
      car.history.push_front(det);
    });

    state.buffer<world_model::Human>().upsert("1", default_human, [](world_model::Human & human) {
      vision_msgs::msg::Detection3D det;
      det.id = "1";
      human.history.push_front(det);
    });

    REQUIRE(state.buffer<world_model::Car>().size() == 1);
    REQUIRE(state.buffer<world_model::Human>().size() == 1);
  }
}

// LaneletHandler Route Caching Tests

TEST_CASE("LaneletHandler route caching", "[lanelet_handler]")
{
  world_model::LaneletHandler handler;

  SECTION("No active route initially")
  {
    REQUIRE_FALSE(handler.hasActiveRoute());
    REQUIRE(handler.getGoalLaneletId() == -1);
  }

  SECTION("setActiveRoute fails without loaded map")
  {
    bool result = handler.setActiveRoute(100, 200);
    REQUIRE_FALSE(result);
    REQUIRE_FALSE(handler.hasActiveRoute());
  }

  SECTION("clearActiveRoute works")
  {
    handler.clearActiveRoute();
    REQUIRE_FALSE(handler.hasActiveRoute());
    REQUIRE(handler.getGoalLaneletId() == -1);
  }

  SECTION("getShortestRoute returns error without loaded map")
  {
    geometry_msgs::msg::Point pt;
    pt.x = 0.0;
    pt.y = 0.0;

    auto response = handler.getShortestRoute(pt);

    REQUIRE_FALSE(response.success);
    REQUIRE(response.error_message == "map_not_loaded");
  }
}

// Service Integration Tests (with ROS executor)

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "SetRoute and GetShortestRoute services", "[services]")
{
  // Create client nodes
  auto set_route_client = std::make_shared<rclcpp::Node>("set_route_client");
  auto get_route_client = std::make_shared<rclcpp::Node>("get_route_client");

  add_node(set_route_client);
  add_node(get_route_client);
  start_spinning();

  auto set_route_cli = set_route_client->create_client<lanelet_msgs::srv::SetRoute>("set_route");
  auto get_route_cli = get_route_client->create_client<lanelet_msgs::srv::GetShortestRoute>("get_route");

  SECTION("SetRoute service client can be created")
  {
    REQUIRE(set_route_cli != nullptr);
  }

  SECTION("GetShortestRoute service client can be created")
  {
    REQUIRE(get_route_cli != nullptr);
  }
}

// WorldObjects Publisher Tests

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "WorldObjects subscriber receives messages", "[world_objects]")
{
  auto sub_node = std::make_shared<wato::test::SubscriberTestNode<world_model_msgs::msg::WorldObjectArray>>(
    "world_objects", "world_objects_subscriber");

  add_node(sub_node);
  start_spinning();

  SECTION("Subscriber node is created")
  {
    REQUIRE(sub_node->get_name() == std::string("world_objects_subscriber"));
  }

  SECTION("No messages received initially")
  {
    REQUIRE(sub_node->get_message_count() == 0);
  }
}

// Detection Processing Tests

TEST_CASE("Detection3D to Entity conversion", "[detection]")
{
  world_model::EntityBuffer<world_model::Human> buffer;

  SECTION("Detection with valid ID is stored")
  {
    vision_msgs::msg::Detection3D det;
    det.id = "123";
    det.header.stamp.sec = 100;
    det.header.stamp.nanosec = 500000000;
    det.header.frame_id = "base_link";
    det.bbox.center.position.x = 5.0;
    det.bbox.center.position.y = 10.0;
    det.bbox.center.position.z = 1.0;
    det.bbox.size.x = 0.5;
    det.bbox.size.y = 0.5;
    det.bbox.size.z = 1.8;

    world_model::Human default_human;
    buffer.upsert("123", default_human, [&det](world_model::Human & human) { human.history.push_front(det); });

    auto retrieved = buffer.get("123");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->id() == "123");
    REQUIRE(retrieved->frameId() == "base_link");
    REQUIRE(retrieved->pose().position.x == Catch::Approx(5.0));
    REQUIRE(retrieved->pose().position.y == Catch::Approx(10.0));
    REQUIRE(retrieved->size().x == Catch::Approx(0.5));
    REQUIRE(retrieved->size().z == Catch::Approx(1.8));
  }

  SECTION("History preserves multiple detections")
  {
    world_model::Human default_human;

    for (int i = 0; i < 5; ++i) {
      buffer.upsert("1", default_human, [i](world_model::Human & human) {
        vision_msgs::msg::Detection3D det;
        det.id = "1";
        det.header.stamp.sec = i;
        det.bbox.center.position.x = static_cast<double>(i);
        human.history.push_front(det);
      });
    }

    auto retrieved = buffer.get("1");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->history.size() == 5);

    // Most recent is at front
    REQUIRE(retrieved->history.front().header.stamp.sec == 4);
    REQUIRE(retrieved->history.back().header.stamp.sec == 0);
  }
}

// Entity Type Classification Tests

TEST_CASE("Entity type identification", "[entity_type]")
{
  SECTION("Car entity type")
  {
    world_model::Car car;
    REQUIRE(car.type() == world_model::EntityType::CAR);
  }

  SECTION("Human entity type")
  {
    world_model::Human human;
    REQUIRE(human.type() == world_model::EntityType::HUMAN);
  }

  SECTION("Bicycle entity type")
  {
    world_model::Bicycle bicycle;
    REQUIRE(bicycle.type() == world_model::EntityType::BICYCLE);
  }

  SECTION("Motorcycle entity type")
  {
    world_model::Motorcycle motorcycle;
    REQUIRE(motorcycle.type() == world_model::EntityType::MOTORCYCLE);
  }
}

// Thread Safety Tests

TEST_CASE("EntityBuffer thread safety", "[thread_safety]")
{
  world_model::EntityBuffer<world_model::Car> buffer;

  SECTION("Concurrent upserts don't corrupt data")
  {
    constexpr int num_threads = 4;
    constexpr int ops_per_thread = 100;
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&buffer, t]() {
        for (int i = 0; i < ops_per_thread; ++i) {
          std::string id = std::to_string(t * ops_per_thread + i);
          world_model::Car default_car;
          buffer.upsert(id, default_car, [&id](world_model::Car & car) {
            vision_msgs::msg::Detection3D det;
            det.id = id;
            det.bbox.center.position.x = 0.0;
            car.history.push_front(det);
          });
        }
      });
    }

    for (auto & t : threads) {
      t.join();
    }

    REQUIRE(buffer.size() == num_threads * ops_per_thread);
  }

  SECTION("Concurrent reads don't block")
  {
    // Populate buffer
    for (int i = 0; i < 100; ++i) {
      std::string sid = std::to_string(i);
      world_model::Car default_car;
      buffer.upsert(sid, default_car, [sid](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = sid;
        car.history.push_front(det);
      });
    }

    constexpr int num_readers = 4;
    std::atomic<int> successful_reads{0};
    std::vector<std::thread> readers;

    for (int t = 0; t < num_readers; ++t) {
      readers.emplace_back([&buffer, &successful_reads]() {
        for (int i = 0; i < 100; ++i) {
          auto entity = buffer.get(std::to_string(i));
          if (entity.has_value()) {
            successful_reads++;
          }
        }
      });
    }

    for (auto & t : readers) {
      t.join();
    }

    REQUIRE(successful_reads == num_readers * 100);
  }
}

// WorldStateReader/Writer Tests

TEST_CASE("WorldStateReader provides const access", "[world_state_accessor]")
{
  world_model::WorldState state;

  // Populate via direct access
  world_model::Car default_car;
  state.buffer<world_model::Car>().upsert("1", default_car, [](world_model::Car & car) {
    vision_msgs::msg::Detection3D det;
    det.id = "1";
    det.bbox.center.position.x = 42.0;
    car.history.push_front(det);
  });

  world_model::WorldStateReader reader(&state);

  SECTION("Reader can access buffers")
  {
    const auto & buffer = reader.buffer<world_model::Car>();
    REQUIRE(buffer.size() == 1);

    auto car = buffer.get("1");
    REQUIRE(car.has_value());
    REQUIRE(car->pose().position.x == Catch::Approx(42.0));
  }

  SECTION("Reader sees all entity types")
  {
    REQUIRE(reader.buffer<world_model::Human>().size() == 0);
    REQUIRE(reader.buffer<world_model::Bicycle>().size() == 0);
    REQUIRE(reader.buffer<world_model::TrafficLight>().size() == 0);
  }
}

TEST_CASE("WorldStateWriter provides mutable access", "[world_state_accessor]")
{
  world_model::WorldState state;
  world_model::WorldStateWriter writer(&state);

  SECTION("Writer can modify buffers")
  {
    world_model::Human default_human;
    writer.buffer<world_model::Human>().upsert("99", default_human, [](world_model::Human & human) {
      vision_msgs::msg::Detection3D det;
      det.id = "99";
      det.bbox.center.position.y = 123.0;
      human.history.push_front(det);
    });

    REQUIRE(writer.buffer<world_model::Human>().size() == 1);

    auto human = writer.buffer<world_model::Human>().get("99");
    REQUIRE(human.has_value());
    REQUIRE(human->pose().position.y == Catch::Approx(123.0));
  }

  SECTION("Writer const access works")
  {
    const auto & const_buffer = const_cast<const world_model::WorldStateWriter &>(writer).buffer<world_model::Car>();
    REQUIRE(const_buffer.size() == 0);
  }
}

// TrafficLight Entity Tests

TEST_CASE("TrafficLight entity buffer", "[traffic_light]")
{
  world_model::EntityBuffer<world_model::TrafficLight> buffer;

  SECTION("TrafficLight entity can be created")
  {
    world_model::TrafficLight default_tl;
    buffer.upsert("500", default_tl, [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection3D det;
      det.id = "500";
      det.header.stamp.sec = 10;
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::RED;
      tl.confidence = 0.95;
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get("500");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->state == world_model::TrafficLightState::RED);
    REQUIRE(retrieved->confidence == Catch::Approx(0.95));
  }

  SECTION("TrafficLight state can be updated")
  {
    world_model::TrafficLight default_tl;

    // Initial: RED
    buffer.upsert("600", default_tl, [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection3D det;
      det.id = "600";
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::RED;
    });

    // Update: GREEN
    buffer.modify("600", [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection3D det;
      det.id = "600";
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::GREEN;
    });

    auto retrieved = buffer.get("600");
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->state == world_model::TrafficLightState::GREEN);
    REQUIRE(retrieved->history.size() == 2);
  }
}

// Cleanup Logic Tests

TEST_CASE("Entity pruning based on timestamp", "[cleanup]")
{
  world_model::EntityBuffer<world_model::Car> buffer;

  SECTION("Prune removes stale entities")
  {
    rclcpp::Time now = rclcpp::Time(1000, 0, RCL_ROS_TIME);

    // Add entities with different timestamps
    for (int i = 0; i < 5; ++i) {
      std::string sid = std::to_string(i);
      world_model::Car default_car;
      buffer.upsert(sid, default_car, [sid, i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = sid;
        // Entity 0-2 are old, 3-4 are recent
        det.header.stamp.sec = (i < 3) ? 990 : 999;
        car.history.push_front(det);
      });
    }

    REQUIRE(buffer.size() == 5);

    // Prune entities older than 5 seconds
    buffer.prune([&now](const world_model::Car & car) { return (now - car.timestamp()).seconds() > 5.0; });

    REQUIRE(buffer.size() == 2);
    REQUIRE_FALSE(buffer.get("0").has_value());
    REQUIRE_FALSE(buffer.get("1").has_value());
    REQUIRE_FALSE(buffer.get("2").has_value());
    REQUIRE(buffer.get("3").has_value());
    REQUIRE(buffer.get("4").has_value());
  }
}

// Entity Empty State Tests

TEST_CASE("Entity empty state", "[entity]")
{
  SECTION("Entity3D is empty without history")
  {
    world_model::Car car;
    REQUIRE(car.empty());
    REQUIRE(car.history.empty());
  }

  SECTION("Entity3D is not empty with history")
  {
    world_model::Car car;
    vision_msgs::msg::Detection3D det;
    det.id = "1";
    car.history.push_front(det);

    REQUIRE_FALSE(car.empty());
  }

  SECTION("TrafficLight is empty without history")
  {
    world_model::TrafficLight tl;
    REQUIRE(tl.empty());
    REQUIRE(tl.history.empty());
  }

  SECTION("TrafficLight is not empty with history")
  {
    world_model::TrafficLight tl;
    vision_msgs::msg::Detection3D det;
    det.id = "1";
    tl.history.push_front(det);

    REQUIRE_FALSE(tl.empty());
  }
}

// ForEach Tests

TEST_CASE("EntityBuffer forEach operations", "[entity_buffer]")
{
  world_model::EntityBuffer<world_model::Car> buffer;

  // Populate
  for (int i = 0; i < 3; ++i) {
    std::string sid = std::to_string(i);
    world_model::Car default_car;
    buffer.upsert(sid, default_car, [sid, i](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = sid;
      det.bbox.center.position.x = static_cast<double>(i);
      car.history.push_front(det);
    });
  }

  SECTION("forEach modifies all entities")
  {
    buffer.forEach([](world_model::Car & car) { car.lanelet_id = 999; });

    for (int i = 0; i < 3; ++i) {
      auto car = buffer.get(std::to_string(i));
      REQUIRE(car.has_value());
      REQUIRE(car->lanelet_id.has_value());
      REQUIRE(*car->lanelet_id == 999);
    }
  }

  SECTION("forEachConst reads all entities")
  {
    int count = 0;
    double sum = 0.0;

    buffer.forEachConst([&count, &sum](const world_model::Car & car) {
      count++;
      sum += car.pose().position.x;
    });

    REQUIRE(count == 3);
    REQUIRE(sum == Catch::Approx(3.0));  // 0 + 1 + 2
  }
}

// GetByLanelet Tests

TEST_CASE("EntityBuffer getByLanelet", "[entity_buffer]")
{
  world_model::EntityBuffer<world_model::Car> buffer;

  SECTION("Returns entities on specified lanelet")
  {
    for (int i = 0; i < 5; ++i) {
      std::string sid = std::to_string(i);
      world_model::Car default_car;
      buffer.upsert(sid, default_car, [sid, i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = sid;
        car.history.push_front(det);
        car.lanelet_id = (i % 2 == 0) ? 100 : 200;
      });
    }

    auto on_lanelet_100 = buffer.getByLanelet(100);
    auto on_lanelet_200 = buffer.getByLanelet(200);
    auto on_lanelet_999 = buffer.getByLanelet(999);

    REQUIRE(on_lanelet_100.size() == 3);  // 0, 2, 4
    REQUIRE(on_lanelet_200.size() == 2);  // 1, 3
    REQUIRE(on_lanelet_999.empty());
  }
}

// ============================================================
// Curvature Computation Tests
// ============================================================

// Helper: build a lanelet::Lanelet from left/right boundary point lists
static lanelet::Lanelet makeLanelet(
  const std::vector<std::tuple<double, double, double>> & left_pts,
  const std::vector<std::tuple<double, double, double>> & right_pts)
{
  lanelet::Points3d left, right;
  for (const auto & [x, y, z] : left_pts) {
    left.emplace_back(lanelet::utils::getId(), x, y, z);
  }
  for (const auto & [x, y, z] : right_pts) {
    right.emplace_back(lanelet::utils::getId(), x, y, z);
  }

  lanelet::LineString3d left_ls(lanelet::utils::getId(), left);
  lanelet::LineString3d right_ls(lanelet::utils::getId(), right);
  return lanelet::Lanelet(lanelet::utils::getId(), left_ls, right_ls);
}

TEST_CASE("toLaneletMsg populates centerline curvature", "[curvature]")
{
  world_model::LaneletHandler handler;  // no map loaded — toLaneletMsg is still callable

  SECTION("Curvature array length matches centerline length")
  {
    auto ll =
      makeLanelet({{0, 1, 0}, {10, 1, 0}, {20, 1, 0}, {30, 1, 0}}, {{0, -1, 0}, {10, -1, 0}, {20, -1, 0}, {30, -1, 0}});

    auto msg = handler.toLaneletMsg(ll);

    REQUIRE_FALSE(msg.centerline.empty());
    REQUIRE(msg.centerline_curvature.size() == msg.centerline.size());
  }

  SECTION("Straight lanelet has near-zero curvature")
  {
    auto ll = makeLanelet(
      {{0, 1, 0}, {10, 1, 0}, {20, 1, 0}, {30, 1, 0}, {40, 1, 0}},
      {{0, -1, 0}, {10, -1, 0}, {20, -1, 0}, {30, -1, 0}, {40, -1, 0}});

    auto msg = handler.toLaneletMsg(ll);

    for (size_t i = 0; i < msg.centerline_curvature.size(); ++i) {
      REQUIRE(std::abs(msg.centerline_curvature[i]) < 1e-6);
    }
  }

  SECTION("Curved lanelet has non-zero curvature with consistent sign")
  {
    // Quarter circle curving left, radius ~10 for centerline
    // Left boundary at r=11, right boundary at r=9
    constexpr int N = 5;
    std::vector<std::tuple<double, double, double>> left_pts, right_pts;
    for (int i = 0; i < N; ++i) {
      double theta = i * (M_PI / 2.0) / (N - 1);
      left_pts.emplace_back(11.0 * std::cos(theta), 11.0 * std::sin(theta), 0.0);
      right_pts.emplace_back(9.0 * std::cos(theta), 9.0 * std::sin(theta), 0.0);
    }

    auto ll = makeLanelet(left_pts, right_pts);
    auto msg = handler.toLaneletMsg(ll);

    REQUIRE(msg.centerline_curvature.size() == msg.centerline.size());

    // Interior points should have positive curvature (curving left)
    for (size_t i = 1; i + 1 < msg.centerline_curvature.size(); ++i) {
      REQUIRE(msg.centerline_curvature[i] > 0.0);
      // Approximate check: curvature should be near 1/10 = 0.1
      REQUIRE(msg.centerline_curvature[i] == Catch::Approx(0.1).margin(0.02));
    }

    // Endpoints copy neighbor values
    REQUIRE(msg.centerline_curvature.front() == Catch::Approx(msg.centerline_curvature[1]));
    REQUIRE(
      msg.centerline_curvature.back() == Catch::Approx(msg.centerline_curvature[msg.centerline_curvature.size() - 2]));
  }
}

// ============================================================
// Arc-Length Distance Tests
// ============================================================

// Helper: compute arc-length from closest_idx to end of centerline
// (mirrors the logic in lane_context_publisher.hpp updateDynamicContext)
static double computeArcLength(const std::vector<geometry_msgs::msg::Point> & cl, size_t closest_idx)
{
  double arc_len = 0.0;
  for (size_t i = closest_idx; i + 1 < cl.size(); ++i) {
    double dx = cl[i + 1].x - cl[i].x;
    double dy = cl[i + 1].y - cl[i].y;
    arc_len += std::sqrt(dx * dx + dy * dy);
  }
  return arc_len;
}

static geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

TEST_CASE("Arc-length distance computation", "[arc_length]")
{
  SECTION("Straight line arc-length equals Euclidean distance")
  {
    std::vector<geometry_msgs::msg::Point> cl = {makePoint(0, 0), makePoint(10, 0), makePoint(20, 0), makePoint(30, 0)};

    // From start: total length = 30
    REQUIRE(computeArcLength(cl, 0) == Catch::Approx(30.0));
    // From index 1: remaining = 20
    REQUIRE(computeArcLength(cl, 1) == Catch::Approx(20.0));
    // From index 2: remaining = 10
    REQUIRE(computeArcLength(cl, 2) == Catch::Approx(10.0));
  }

  SECTION("Arc-length from last point is zero")
  {
    std::vector<geometry_msgs::msg::Point> cl = {makePoint(0, 0), makePoint(5, 0), makePoint(10, 0)};

    REQUIRE(computeArcLength(cl, 2) == Catch::Approx(0.0));
  }

  SECTION("L-shaped path arc-length exceeds straight-line distance")
  {
    // Path goes right 10m then up 10m — arc-length = 20, Euclidean end-to-end ~14.14
    std::vector<geometry_msgs::msg::Point> cl = {makePoint(0, 0), makePoint(10, 0), makePoint(10, 10)};

    double arc = computeArcLength(cl, 0);
    REQUIRE(arc == Catch::Approx(20.0));

    // Euclidean from start to end
    double euclidean = std::sqrt(10.0 * 10.0 + 10.0 * 10.0);
    REQUIRE(arc > euclidean);
  }

  SECTION("Curved path arc-length exceeds chord distance")
  {
    // Quarter circle r=10: arc-length = pi*10/2 ≈ 15.71
    constexpr int N = 50;  // enough points for good approximation
    std::vector<geometry_msgs::msg::Point> cl;
    for (int i = 0; i < N; ++i) {
      double theta = i * (M_PI / 2.0) / (N - 1);
      cl.push_back(makePoint(10.0 * std::cos(theta), 10.0 * std::sin(theta)));
    }

    double arc = computeArcLength(cl, 0);
    double expected = M_PI * 10.0 / 2.0;  // ≈ 15.708

    REQUIRE(arc == Catch::Approx(expected).margin(0.05));

    // Chord distance from (10,0) to (0,10)
    double chord = std::sqrt(10.0 * 10.0 + 10.0 * 10.0);
    REQUIRE(arc > chord);
  }
}

// ============================================================
// findCurrentLaneletId with BFS Hint Tests
// ============================================================

TEST_CASE("findCurrentLaneletId with BFS hint", "[lanelet_handler][bfs_hint]")
{
  world_model::LaneletHandler handler;  // no map loaded

  geometry_msgs::msg::Point pt;
  pt.x = 0.0;
  pt.y = 0.0;

  SECTION("Returns nullopt without map — no hint")
  {
    auto result = handler.findCurrentLaneletId(pt, 0.0);
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Returns nullopt without map — with hint")
  {
    auto result = handler.findCurrentLaneletId(pt, 0.0, 10.0, 15.0, 42);
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Returns nullopt without map — nullopt hint explicit")
  {
    auto result = handler.findCurrentLaneletId(pt, 0.0, 10.0, 15.0, std::nullopt);
    REQUIRE_FALSE(result.has_value());
  }
}

// ============================================================
// getLaneletAhead with BFS Hint Tests
// ============================================================

TEST_CASE("getLaneletAhead with BFS hint", "[lanelet_handler][bfs_hint]")
{
  world_model::LaneletHandler handler;  // no map loaded

  geometry_msgs::msg::Point pt;
  pt.x = 0.0;
  pt.y = 0.0;

  SECTION("Returns default result without map — no hint")
  {
    auto msg = handler.getLaneletAhead(pt, 0.0, 50.0);
    REQUIRE(msg.current_lanelet_id == -1);
    REQUIRE(msg.lanelets.empty());
  }

  SECTION("Returns default result without map — with hint")
  {
    auto msg = handler.getLaneletAhead(pt, 0.0, 50.0, 42);
    REQUIRE(msg.current_lanelet_id == -1);
    REQUIRE(msg.lanelets.empty());
  }

  SECTION("Returns default result without map — nullopt hint explicit")
  {
    auto msg = handler.getLaneletAhead(pt, 0.0, 50.0, std::nullopt);
    REQUIRE(msg.current_lanelet_id == -1);
    REQUIRE(msg.lanelets.empty());
  }
}
