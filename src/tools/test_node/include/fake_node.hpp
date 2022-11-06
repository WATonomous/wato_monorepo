#ifndef FAKE_NODE_HPP_
#define FAKE_NODE_HPP_

namespace tools
{
namespace test_node
{

#include "rclcpp/rclcpp.hpp"

class FakeNode : public rclcpp::Node
{
public:
  FakeNode();

};

}  // namespace test_node
} // namespace tools

#endif  // FAKE_NODE_HPP_
