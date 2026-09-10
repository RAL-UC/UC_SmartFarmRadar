// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_HPP_

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
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msg__msg__RadarCartesian __attribute__((deprecated))
#else
# define DEPRECATED__radar_msg__msg__RadarCartesian __declspec(deprecated)
#endif

namespace radar_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarCartesian_
{
  using Type = RadarCartesian_<ContainerAllocator>;

  explicit RadarCartesian_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gps_frame = "";
      this->robot_pose_id = 0ul;
    }
  }

  explicit RadarCartesian_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gps_frame(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gps_frame = "";
      this->robot_pose_id = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _stamps_type =
    std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>>;
  _stamps_type stamps;
  using _x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _x_type x;
  using _y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _y_type y;
  using _z_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _z_type z;
  using _gps_e_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_e_type gps_e;
  using _gps_n_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_n_type gps_n;
  using _gps_alt_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_alt_type gps_alt;
  using _gps_qx_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_qx_type gps_qx;
  using _gps_qy_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_qy_type gps_qy;
  using _gps_qz_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_qz_type gps_qz;
  using _gps_qw_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gps_qw_type gps_qw;
  using _gps_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gps_frame_type gps_frame;
  using _robot_pose_id_type =
    uint32_t;
  _robot_pose_id_type robot_pose_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__stamps(
    const std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>> & _arg)
  {
    this->stamps = _arg;
    return *this;
  }
  Type & set__x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__gps_e(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_e = _arg;
    return *this;
  }
  Type & set__gps_n(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_n = _arg;
    return *this;
  }
  Type & set__gps_alt(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_alt = _arg;
    return *this;
  }
  Type & set__gps_qx(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_qx = _arg;
    return *this;
  }
  Type & set__gps_qy(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_qy = _arg;
    return *this;
  }
  Type & set__gps_qz(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_qz = _arg;
    return *this;
  }
  Type & set__gps_qw(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gps_qw = _arg;
    return *this;
  }
  Type & set__gps_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gps_frame = _arg;
    return *this;
  }
  Type & set__robot_pose_id(
    const uint32_t & _arg)
  {
    this->robot_pose_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msg::msg::RadarCartesian_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msg::msg::RadarCartesian_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::RadarCartesian_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msg::msg::RadarCartesian_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msg__msg__RadarCartesian
    std::shared_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msg__msg__RadarCartesian
    std::shared_ptr<radar_msg::msg::RadarCartesian_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarCartesian_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->stamps != other.stamps) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->gps_e != other.gps_e) {
      return false;
    }
    if (this->gps_n != other.gps_n) {
      return false;
    }
    if (this->gps_alt != other.gps_alt) {
      return false;
    }
    if (this->gps_qx != other.gps_qx) {
      return false;
    }
    if (this->gps_qy != other.gps_qy) {
      return false;
    }
    if (this->gps_qz != other.gps_qz) {
      return false;
    }
    if (this->gps_qw != other.gps_qw) {
      return false;
    }
    if (this->gps_frame != other.gps_frame) {
      return false;
    }
    if (this->robot_pose_id != other.robot_pose_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarCartesian_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarCartesian_

// alias to use template instance with default allocator
using RadarCartesian =
  radar_msg::msg::RadarCartesian_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__STRUCT_HPP_
