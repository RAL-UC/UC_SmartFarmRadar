// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_HPP_

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
# define DEPRECATED__radar_msg__msg__RadarData __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__msg__RadarData __declspec(deprecated)
#endif

namespace radar_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarData_
{
  using Type = RadarData_<ContainerAllocator>;

  explicit RadarData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rows = 0ul;
      this->cols = 0ul;
      this->dtype = "";
    }
  }

  explicit RadarData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    dtype(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rows = 0ul;
      this->cols = 0ul;
      this->dtype = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rows_type =
    uint32_t;
  _rows_type rows;
  using _cols_type =
    uint32_t;
  _cols_type cols;
  using _dtype_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dtype_type dtype;
  using _data_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rows(
    const uint32_t & _arg)
  {
    this->rows = _arg;
    return *this;
  }
  Type & set__cols(
    const uint32_t & _arg)
  {
    this->cols = _arg;
    return *this;
  }
  Type & set__dtype(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dtype = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::msg::RadarData_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::msg::RadarData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::msg::RadarData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::msg::RadarData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::RadarData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::RadarData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::RadarData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::RadarData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::msg::RadarData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::msg::RadarData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__msg__RadarData
    std::shared_ptr<radar_msg::msg::RadarData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__msg__RadarData
    std::shared_ptr<radar_msg::msg::RadarData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rows != other.rows) {
      return false;
    }
    if (this->cols != other.cols) {
      return false;
    }
    if (this->dtype != other.dtype) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarData_

// alias to use template instance with default allocator
using RadarData =
  radar_msg::msg::RadarData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_DATA__STRUCT_HPP_
