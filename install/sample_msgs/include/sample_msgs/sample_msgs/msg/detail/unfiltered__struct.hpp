// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_msgs:msg/Unfiltered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_msgs__msg__Unfiltered __attribute__((deprecated))
#else
# define DEPRECATED__sample_msgs__msg__Unfiltered __declspec(deprecated)
#endif

namespace sample_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Unfiltered_
{
  using Type = Unfiltered_<ContainerAllocator>;

  explicit Unfiltered_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
      this->timestamp = 0ll;
      this->valid = false;
    }
  }

  explicit Unfiltered_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
      this->timestamp = 0ll;
      this->valid = false;
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;
  using _valid_type =
    bool;
  _valid_type valid;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__timestamp(
    const int64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_msgs::msg::Unfiltered_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_msgs::msg::Unfiltered_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Unfiltered_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Unfiltered_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_msgs__msg__Unfiltered
    std::shared_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_msgs__msg__Unfiltered
    std::shared_ptr<sample_msgs::msg::Unfiltered_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Unfiltered_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const Unfiltered_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Unfiltered_

// alias to use template instance with default allocator
using Unfiltered =
  sample_msgs::msg::Unfiltered_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__UNFILTERED__STRUCT_HPP_
