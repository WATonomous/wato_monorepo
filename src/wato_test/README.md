# wato_test

A ROS 2 testing framework built on Catch2 that provides test fixtures and utilities for testing ROS nodes, services, and communication patterns.

## Features

- **TestExecutorFixture**: Manages ROS executor lifecycle for tests
- **Test Nodes**: Pre-built nodes for testing publishers, subscribers, services, and clients
- **CMake Integration**: Simple `add_wato_test()` function for creating tests
- **Catch2 Integration**: Built on Catch2 for modern C++ testing

## Quick Start

```cpp
#include <catch2/catch.hpp>
#include <wato_test/wato_test.hpp>

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "My ROS Test", "[ros]") {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  add_node(node);
  start_spinning();

  // Your test code here as sections
  SECTION("My test") {
    REQUIRE(node->get_name() == "test_node");
  }
}
```

## Package.xml Usage

```xml
<test_depend>wato_test</test_depend>
```

## CMake Usage

```cmake
find_package(wato_test REQUIRED)

add_wato_test(my_test
  test/my_test.cpp
  LIBRARIES
    my_package::my_library
    std_msgs::std_msgs
)
```
