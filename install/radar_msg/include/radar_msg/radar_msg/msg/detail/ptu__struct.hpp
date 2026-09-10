// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msg:msg/Ptu.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__PTU__STRUCT_HPP_
#define RADAR_MSG__MSG__DETAIL__PTU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__msg__Ptu __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__msg__Ptu __declspec(deprecated)
#endif

namespace radar_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Ptu_
{
  using Type = Ptu_<ContainerAllocator>;

  explicit Ptu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pan_deg = 0l;
      this->tilt_deg = 0l;
    }
  }

  explicit Ptu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pan_deg = 0l;
      this->tilt_deg = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pan_deg_type =
    int32_t;
  _pan_deg_type pan_deg;
  using _tilt_deg_type =
    int32_t;
  _tilt_deg_type tilt_deg;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pan_deg(
    const int32_t & _arg)
  {
    this->pan_deg = _arg;
    return *this;
  }
  Type & set__tilt_deg(
    const int32_t & _arg)
  {
    this->tilt_deg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::msg::Ptu_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::msg::Ptu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::msg::Ptu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::msg::Ptu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::Ptu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::Ptu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::Ptu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::Ptu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::msg::Ptu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::msg::Ptu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__msg__Ptu
    std::shared_ptr<radar_msg::msg::Ptu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__msg__Ptu
    std::shared_ptr<radar_msg::msg::Ptu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Ptu_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pan_deg != other.pan_deg) {
      return false;
    }
    if (this->tilt_deg != other.tilt_deg) {
      return false;
    }
    return true;
  }
  bool operator!=(const Ptu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Ptu_

// alias to use template instance with default allocator
using Ptu =
  radar_msg::msg::Ptu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__PTU__STRUCT_HPP_
