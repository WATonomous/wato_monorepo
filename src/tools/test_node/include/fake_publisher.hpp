#ifndef FAKE_PUBLISHER_HPP_
#define FAKE_PUBLISHER_HPP_

namespace tools
{
namespace test_node
{

#include "rclcpp/rclcpp.hpp"

template <typename MessageT>
class FakePublisher<MessageT> : public rclcpp::Publisher<MessageT>
{
public:
  FakePublisher();

  void publish(const MessageT & msg) override;

  int enqueued_size() const;
  std::vector<MessageT> enqueued_messages() const;

private:
  std::vector<MessageT> msgs_;
};

template <typename MessageT>
FakePublisher<MessageT>::FakePublisher()
{}

template <typename MessageT>
void FakePublisher<MessageT>::publish(const MessageT & msg)
{
  msgs_.push_back(msg);
}

template <typename MessageT>
int FakePublisher<MessageT>::enqueued_size() const
{
  return msgs_.size();
}

template <typename MessageT>
std::vector<T> FakePublisher<MessageT>::enqueued_messages() const
{
  return msgs_;
}

} // namespace test_node
} // namespace tools

#endif  // FAKE_PUBLISHER_HPP_
