# Developing wato_test

## Building

```bash
colcon build --packages-select wato_test
```

## Running Tests

```bash
colcon test --packages-select wato_test
colcon test-result --verbose
```

## Adding New Features

### Test Fixtures
This includes code that follows the definition of a fixture as per catch2.

1. Add header to `include/test_fixtures/`
2. Add implementation to `src/` if needed
3. Include in `include/wato_test/wato_test.hpp`

### Test Nodes
This includes code that acts as helper ros nodes for testing. They should be inherently event-driven nodes (do not introduce arbitrary sleeps).

1. Add header to `include/test_nodes/`
2. Include in main header
3. Keep nodes header-only when possible

#### Discovery Waiting

Use `wait_for_subscribers()` / `wait_for_publishers()` instead of hardcoded `sleep_for` calls to wait for DDS discovery. These poll the DDS middleware's discovery state (independent of the ROS executor) and return as soon as matching is complete, or `false` on timeout (default 10s).

```cpp
start_spinning();
REQUIRE(pub->wait_for_subscribers(1));
REQUIRE(sub->wait_for_publishers(1));
```

#### Lifecycle Node Testing

When testing lifecycle nodes with `SECTION`s that override parameters, set parameters **before** activation to avoid racing `set_parameter()` on the test thread with node callbacks on the executor thread:

```cpp
pid_node->trigger_transition(Transition::TRANSITION_CONFIGURE);
// ... create test nodes ...

SECTION("My test") {
  node->set_parameter(rclcpp::Parameter("my_param", 42.0));  // safe: no timer yet
  node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  start_spinning();
  REQUIRE(pub->wait_for_subscribers(1));
  // ...
}
```

## CMake Function

The `add_wato_test()` function automatically:
- Links Catch2::Catch2WithMain
- Links ROS dependencies (rclcpp, rclcpp_lifecycle)
- Configures ament test registration
- Sets up JUnit XML output
