// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_msgs:msg/FilteredArray.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'packets'
#include "sample_msgs/msg/detail/filtered__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sample_msgs__msg__FilteredArray __attribute__((deprecated))
#else
# define DEPRECATED__sample_msgs__msg__FilteredArray __declspec(deprecated)
#endif

namespace sample_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FilteredArray_
{
  using Type = FilteredArray_<ContainerAllocator>;

  explicit FilteredArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FilteredArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _packets_type =
    std::vector<sample_msgs::msg::Filtered_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sample_msgs::msg::Filtered_<ContainerAllocator>>>;
  _packets_type packets;

  // setters for named parameter idiom
  Type & set__packets(
    const std::vector<sample_msgs::msg::Filtered_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sample_msgs::msg::Filtered_<ContainerAllocator>>> & _arg)
  {
    this->packets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_msgs::msg::FilteredArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_msgs::msg::FilteredArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::FilteredArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::FilteredArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_msgs__msg__FilteredArray
    std::shared_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_msgs__msg__FilteredArray
    std::shared_ptr<sample_msgs::msg::FilteredArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredArray_ & other) const
  {
    if (this->packets != other.packets) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredArray_

// alias to use template instance with default allocator
using FilteredArray =
  sample_msgs::msg::FilteredArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__FILTERED_ARRAY__STRUCT_HPP_
