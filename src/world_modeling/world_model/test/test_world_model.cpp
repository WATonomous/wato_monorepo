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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch_all.hpp>
#include <wato_test/wato_test.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "lanelet_msgs/srv/get_shortest_route.hpp"
#include "lanelet_msgs/srv/set_route.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "world_model/interfaces/interface_base.hpp"
#include "world_model/lanelet_handler.hpp"
#include "world_model/types/entity_2d.hpp"
#include "world_model/types/entity_3d.hpp"
#include "world_model/types/entity_buffer.hpp"
#include "world_model/world_state.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

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
    buffer.upsert(42, default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 10.0;
      det.bbox.center.position.y = 20.0;
      car.history.push_front(det);
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get(42);
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->id() == 42);
    REQUIRE(retrieved->pose().position.x == Catch::Approx(10.0));
  }

  SECTION("Upsert updates existing entity")
  {
    world_model::Car default_car;

    // First insert
    buffer.upsert(42, default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 10.0;
      car.history.push_front(det);
    });

    // Update
    buffer.upsert(42, default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "42";
      det.bbox.center.position.x = 30.0;
      car.history.push_front(det);
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get(42);
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->pose().position.x == Catch::Approx(30.0));
    REQUIRE(retrieved->history.size() == 2);
  }

  SECTION("Get returns nullopt for non-existent entity")
  {
    auto retrieved = buffer.get(999);
    REQUIRE_FALSE(retrieved.has_value());
  }

  SECTION("Prune removes entities matching predicate")
  {
    world_model::Car default_car;

    for (int i = 0; i < 5; ++i) {
      buffer.upsert(i, default_car, [i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = std::to_string(i);
        det.bbox.center.position.x = static_cast<double>(i * 10);
        car.history.push_front(det);
      });
    }

    REQUIRE(buffer.size() == 5);

    // Prune entities with x < 25 (removes 0, 1, 2)
    buffer.prune([](const world_model::Car & car) { return car.pose().position.x < 25.0; });

    REQUIRE(buffer.size() == 2);
    REQUIRE_FALSE(buffer.get(0).has_value());
    REQUIRE_FALSE(buffer.get(1).has_value());
    REQUIRE_FALSE(buffer.get(2).has_value());
    REQUIRE(buffer.get(3).has_value());
    REQUIRE(buffer.get(4).has_value());
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

    state.buffer<world_model::Car>().upsert(1, default_car, [](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = "1";
      car.history.push_front(det);
    });

    state.buffer<world_model::Human>().upsert(1, default_human, [](world_model::Human & human) {
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

// DynamicObjects Publisher Tests

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "DynamicObjects subscriber receives messages", "[dynamic_objects]")
{
  auto sub_node = std::make_shared<wato::test::SubscriberTestNode<world_model_msgs::msg::DynamicObjectArray>>(
    "dynamic_objects", "dynamic_objects_subscriber");

  add_node(sub_node);
  start_spinning();

  SECTION("Subscriber node is created")
  {
    REQUIRE(sub_node->get_name() == std::string("dynamic_objects_subscriber"));
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
    buffer.upsert(123, default_human, [&det](world_model::Human & human) { human.history.push_front(det); });

    auto retrieved = buffer.get(123);
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->id() == 123);
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
      buffer.upsert(1, default_human, [i](world_model::Human & human) {
        vision_msgs::msg::Detection3D det;
        det.id = "1";
        det.header.stamp.sec = i;
        det.bbox.center.position.x = static_cast<double>(i);
        human.history.push_front(det);
      });
    }

    auto retrieved = buffer.get(1);
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
          int64_t id = t * ops_per_thread + i;
          world_model::Car default_car;
          buffer.upsert(id, default_car, [id](world_model::Car & car) {
            vision_msgs::msg::Detection3D det;
            det.id = std::to_string(id);
            det.bbox.center.position.x = static_cast<double>(id);
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
      world_model::Car default_car;
      buffer.upsert(i, default_car, [i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = std::to_string(i);
        car.history.push_front(det);
      });
    }

    constexpr int num_readers = 4;
    std::atomic<int> successful_reads{0};
    std::vector<std::thread> readers;

    for (int t = 0; t < num_readers; ++t) {
      readers.emplace_back([&buffer, &successful_reads]() {
        for (int i = 0; i < 100; ++i) {
          auto entity = buffer.get(i);
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
  state.buffer<world_model::Car>().upsert(1, default_car, [](world_model::Car & car) {
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

    auto car = buffer.get(1);
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
    writer.buffer<world_model::Human>().upsert(99, default_human, [](world_model::Human & human) {
      vision_msgs::msg::Detection3D det;
      det.id = "99";
      det.bbox.center.position.y = 123.0;
      human.history.push_front(det);
    });

    REQUIRE(writer.buffer<world_model::Human>().size() == 1);

    auto human = writer.buffer<world_model::Human>().get(99);
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
    buffer.upsert(500, default_tl, [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection2D det;
      det.id = "500";
      det.header.stamp.sec = 10;
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::RED;
      tl.confidence = 0.95;
    });

    REQUIRE(buffer.size() == 1);

    auto retrieved = buffer.get(500);
    REQUIRE(retrieved.has_value());
    REQUIRE(retrieved->state == world_model::TrafficLightState::RED);
    REQUIRE(retrieved->confidence == Catch::Approx(0.95));
  }

  SECTION("TrafficLight state can be updated")
  {
    world_model::TrafficLight default_tl;

    // Initial: RED
    buffer.upsert(600, default_tl, [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection2D det;
      det.id = "600";
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::RED;
    });

    // Update: GREEN
    buffer.modify(600, [](world_model::TrafficLight & tl) {
      vision_msgs::msg::Detection2D det;
      det.id = "600";
      tl.history.push_front(det);
      tl.state = world_model::TrafficLightState::GREEN;
    });

    auto retrieved = buffer.get(600);
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
      world_model::Car default_car;
      buffer.upsert(i, default_car, [i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = std::to_string(i);
        // Entity 0-2 are old, 3-4 are recent
        det.header.stamp.sec = (i < 3) ? 990 : 999;
        car.history.push_front(det);
      });
    }

    REQUIRE(buffer.size() == 5);

    // Prune entities older than 5 seconds
    buffer.prune([&now](const world_model::Car & car) { return (now - car.timestamp()).seconds() > 5.0; });

    REQUIRE(buffer.size() == 2);
    REQUIRE_FALSE(buffer.get(0).has_value());
    REQUIRE_FALSE(buffer.get(1).has_value());
    REQUIRE_FALSE(buffer.get(2).has_value());
    REQUIRE(buffer.get(3).has_value());
    REQUIRE(buffer.get(4).has_value());
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

  SECTION("Entity2D is empty without history")
  {
    world_model::TrafficLight tl;
    REQUIRE(tl.empty());
    REQUIRE(tl.history.empty());
  }

  SECTION("Entity2D is not empty with history")
  {
    world_model::TrafficLight tl;
    vision_msgs::msg::Detection2D det;
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
    world_model::Car default_car;
    buffer.upsert(i, default_car, [i](world_model::Car & car) {
      vision_msgs::msg::Detection3D det;
      det.id = std::to_string(i);
      det.bbox.center.position.x = static_cast<double>(i);
      car.history.push_front(det);
    });
  }

  SECTION("forEach modifies all entities")
  {
    buffer.forEach([](world_model::Car & car) { car.lanelet_id = 999; });

    for (int i = 0; i < 3; ++i) {
      auto car = buffer.get(i);
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
      world_model::Car default_car;
      buffer.upsert(i, default_car, [i](world_model::Car & car) {
        vision_msgs::msg::Detection3D det;
        det.id = std::to_string(i);
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
