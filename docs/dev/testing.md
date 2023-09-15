# Testing
Our testing practices are motivated by the fact that we are developing a complex system in a field where safety is paramount.
Over time we will introduce bugs and we want to reduce the amount of time spent debugging trivial issues.
Testing is done in two phases. Unit testing, where modules are tested individually and integration testing, where groups of modules are tested together.

## Unit Testing
The goal of unit testing is to ensure that individual modules of the source code behave correctly.
Since the focus of the tests is on a single component it is common to mock external inputs.
This is important for testing ROS2 nodes since they are driven by a subscription model.
The recommended way of structuring ROS2 nodes to facilitate unit testing is to have a class that manages tasks at the ROS system layer such as subscriptions, publishers, and parameters.
Then we have a class that manages the data processing methods that we would like to test.
For instance,
```cpp
class ROSNode : public rclcpp::Node
{
private:
  void callback(...); // Process data by calling methods exposed in logic_

  TestableROSLogic logic_;
  rclcpp::Publisher<...> pub_;
  rclcpp::Subscription<...> sub_;
}

class TestableROSLogic
{
public:
  ... process_data1(...);
  ... process_data2(...);
}
```
For more detailed examples checkout the [Sample C++ ROS2 Nodes](../src/samples/).

For C++ Development ROS2 supports using gtest. Checkout the [Googletest Primer](https://google.github.io/googletest/primer.html) or the [Sample C++ ROS2 Nodes](../src/samples/) for more information.

## Integration Testing
In Progress

## Useful Snippets
**Running Tests**: `colcon test`
- Runs all tests defined in the CMakeLists.txt for the package including the linter
- Running `colcon test --event-handlers console_cohesion+` outputs a detailed summary to the console

**Viewing Test Failures**: `colcon test-result --verbose`
- Generates a detailed summary of failing tests including log statements

**Run Filtered Tests**: `In Progress`

**Running Tests Until Failure**: `colcon test --retest-until-fail <N>`
- Runs the test suite either N times or until a failure is detected
- Useful for identifying flaky tests and debugging race conditions

Read the [colcon docs](https://colcon.readthedocs.io/en/released/reference/verb/test.html) for more information.

## Generating Test Coverage Reports
In Progress