#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "average_filter.hpp"

/**
 * When writing a large number of tests it is desirable to wrap any common
 * setup or cleanup logic in a test fixture. This improves readability and reduces
 * the amount of boilerplate code in each test. For more information checkout
 * https://google.github.io/googletest/primer.html#same-data-multiple-tests
 */
class AverageFilterFixtureTest : public ::testing::Test
{
public:
  void SetUp()
  {}

protected:
  world_modelling::AverageFilter average_filter;
};