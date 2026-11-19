# wato_test

A ROS 2 testing framework built on Catch2 that provides test fixtures and utilities for testing ROS nodes, services, and communication patterns.

## Features

- **TestExecutorFixture**: Manages ROS executor lifecycle for tests
- **Test Nodes**: Pre-built nodes for testing publishers, subscribers, services, and clients
  - **Discovery Waiting**: `PublisherTestNode::wait_for_subscribers()` and `SubscriberTestNode::wait_for_publishers()` poll DDS discovery state with a timeout, replacing fragile hardcoded sleeps
  - **Future-based Messaging**: `SubscriberTestNode::expect_next_message()` returns a `std::future` that resolves when the next message arrives
- **CMake Integration**: Simple `add_wato_test()` function for creating tests
- **Parallel Isolation**: Each `add_wato_test()` executable gets a coordinated ROS domain, including when `ROS_DOMAIN_ID` is inherited from `watod`
- **Catch2 Integration**: Built on Catch2 for modern C++ testing

## Quick Start

```cpp
#include <chrono>
#include <future>

#include <catch2/catch_all.hpp>
#include <std_msgs/msg/string.hpp>
#include <wato_test/wato_test.hpp>

#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "My ROS Test", "[ros]") {
  auto pub = std::make_shared<wato::test::PublisherTestNode<std_msgs::msg::String>>("/topic", "pub");
  auto sub = std::make_shared<wato::test::SubscriberTestNode<std_msgs::msg::String>>("/topic", "sub");
  add_node(pub);
  add_node(sub);
  start_spinning();

  // Wait for DDS discovery (replaces hardcoded sleeps)
  REQUIRE(pub->wait_for_subscribers(1));
  REQUIRE(sub->wait_for_publishers(1));

  SECTION("Publish and receive") {
    auto future = sub->expect_next_message();

    std_msgs::msg::String msg;
    msg.data = "hello";
    pub->publish(msg);

    REQUIRE(future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    REQUIRE(future.get().data == "hello");
  }
}
```

## Avoiding Flaky Tests

If a test passes only after rerunning, check these common timing assumptions:

- **Discovery:** Wait for matched publishers, subscribers, and services before sending inputs. A fixed sleep does not establish readiness.
- **Message ordering:** Register expectations before publishing. For periodic outputs, the next message may reflect an earlier input; use `expect_message(predicate)` to wait for the expected result. Keep predicates nonblocking and do not call back into the subscriber from them.
- **Input processing:** Publishing or receiving a delivery acknowledgment does not mean the receiving callback has finished. Establish processing order before sending an input that depends on an earlier state update.
- **Shared data:** Use promises or mutexes to communicate between the test and callback threads instead of unsynchronized flags.
- **Timeouts and teardown:** Check `future.wait_for(timeout)` before `get()`. Callbacks must also use bounded waits because fixture teardown joins the executor before shutting ROS down.
- **Parallel execution:** Use `add_wato_test()` for automatic domain isolation. Tests launched through other runners need their own isolation to avoid exchanging messages with concurrent tests.

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
