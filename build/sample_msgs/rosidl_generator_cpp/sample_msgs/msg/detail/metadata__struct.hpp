// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_
#define SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_msgs__msg__Metadata __attribute__((deprecated))
#else
# define DEPRECATED__sample_msgs__msg__Metadata __declspec(deprecated)
#endif

namespace sample_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Metadata_
{
  using Type = Metadata_<ContainerAllocator>;

  explicit Metadata_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->compression_method = 0;
      this->creation_date = 0;
    }
  }

  explicit Metadata_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->compression_method = 0;
      this->creation_date = 0;
    }
  }

  // field types and members
  using _version_type =
    int8_t;
  _version_type version;
  using _compression_method_type =
    int8_t;
  _compression_method_type compression_method;
  using _creation_date_type =
    int16_t;
  _creation_date_type creation_date;

  // setters for named parameter idiom
  Type & set__version(
    const int8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__compression_method(
    const int8_t & _arg)
  {
    this->compression_method = _arg;
    return *this;
  }
  Type & set__creation_date(
    const int16_t & _arg)
  {
    this->creation_date = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t DEFAULT =
    0;
  static constexpr int8_t DICTIONARY =
    1;
  static constexpr int8_t RUN_LENGTH =
    2;

  // pointer types
  using RawPtr =
    sample_msgs::msg::Metadata_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_msgs::msg::Metadata_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_msgs::msg::Metadata_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_msgs::msg::Metadata_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Metadata_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Metadata_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_msgs::msg::Metadata_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_msgs::msg::Metadata_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_msgs::msg::Metadata_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_msgs::msg::Metadata_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_msgs__msg__Metadata
    std::shared_ptr<sample_msgs::msg::Metadata_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_msgs__msg__Metadata
    std::shared_ptr<sample_msgs::msg::Metadata_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Metadata_ & other) const
  {
    if (this->version != other.version) {
      return false;
    }
    if (this->compression_method != other.compression_method) {
      return false;
    }
    if (this->creation_date != other.creation_date) {
      return false;
    }
    return true;
  }
  bool operator!=(const Metadata_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Metadata_

// alias to use template instance with default allocator
using Metadata =
  sample_msgs::msg::Metadata_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t Metadata_<ContainerAllocator>::DEFAULT;
template<typename ContainerAllocator>
constexpr int8_t Metadata_<ContainerAllocator>::DICTIONARY;
template<typename ContainerAllocator>
constexpr int8_t Metadata_<ContainerAllocator>::RUN_LENGTH;

}  // namespace msg

}  // namespace sample_msgs

#endif  // SAMPLE_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_
