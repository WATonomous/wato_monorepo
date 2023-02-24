// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_msgs:msg/Filtered.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'metadata'
#include "sample_msgs/msg/detail/metadata__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sample_msgs__msg__Filtered __attribute__((deprecated))
#else
# define DEPRECATED__sample_msgs__msg__Filtered __declspec(deprecated)
#endif

namespace sample_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Filtered_
{
  using Type = Filtered_<ContainerAllocator>;

  explicit Filtered_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : metadata(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_x = 0.0f;
      this->pos_y = 0.0f;
      this->pos_z = 0.0f;
      this->timestamp = 0ll;
    }
  }

  explicit Filtered_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : metadata(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_x = 0.0f;
      this->pos_y = 0.0f;
      this->pos_z = 0.0f;
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _pos_x_type =
    float;
  _pos_x_type pos_x;
  using _pos_y_type =
    float;
  _pos_y_type pos_y;
  using _pos_z_type =
    float;
  _pos_z_type pos_z;
  using _metadata_type =
    sample_msgs::msg::Metadata_<ContainerAllocator>;
  _metadata_type metadata;
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__pos_x(
    const float & _arg)
  {
    this->pos_x = _arg;
    return *this;
  }
  Type & set__pos_y(
    const float & _arg)
  {
    this->pos_y = _arg;
    return *this;
  }
  Type & set__pos_z(
    const float & _arg)
  {
    this->pos_z = _arg;
    return *this;
  }
  Type & set__metadata(
    const sample_msgs::msg::Metadata_<ContainerAllocator> & _arg)
  {
    this->metadata = _arg;
    return *this;
  }
  Type & set__timestamp(
    const int64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_msgs::msg::Filtered_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_msgs::msg::Filtered_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_msgs::msg::Filtered_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_msgs::msg::Filtered_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Filtered_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Filtered_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Filtered_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Filtered_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_msgs::msg::Filtered_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_msgs::msg::Filtered_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_msgs__msg__Filtered
    std::shared_ptr<sample_msgs::msg::Filtered_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_msgs__msg__Filtered
    std::shared_ptr<sample_msgs::msg::Filtered_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Filtered_ & other) const
  {
    if (this->pos_x != other.pos_x) {
      return false;
    }
    if (this->pos_y != other.pos_y) {
      return false;
    }
    if (this->pos_z != other.pos_z) {
      return false;
    }
    if (this->metadata != other.metadata) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Filtered_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Filtered_

// alias to use template instance with default allocator
using Filtered =
  sample_msgs::msg::Filtered_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED__STRUCT_HPP_
