// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msg:msg/Ptu.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__PTU__BUILDER_HPP_
#define RADAR_MSG__MSG__DETAIL__PTU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msg/msg/detail/ptu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msg
{

namespace msg
{

namespace builder
{

class Init_Ptu_tilt_deg
{
public:
  explicit Init_Ptu_tilt_deg(::radar_msg::msg::Ptu & msg)
  : msg_(msg)
  {}
  ::radar_msg::msg::Ptu tilt_deg(::radar_msg::msg::Ptu::_tilt_deg_type arg)
  {
    msg_.tilt_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::msg::Ptu msg_;
};

class Init_Ptu_pan_deg
{
public:
  explicit Init_Ptu_pan_deg(::radar_msg::msg::Ptu & msg)
  : msg_(msg)
  {}
  Init_Ptu_tilt_deg pan_deg(::radar_msg::msg::Ptu::_pan_deg_type arg)
  {
    msg_.pan_deg = std::move(arg);
    return Init_Ptu_tilt_deg(msg_);
  }

private:
  ::radar_msg::msg::Ptu msg_;
};

class Init_Ptu_header
{
public:
  Init_Ptu_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ptu_pan_deg header(::radar_msg::msg::Ptu::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Ptu_pan_deg(msg_);
  }

private:
  ::radar_msg::msg::Ptu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::msg::Ptu>()
{
  return radar_msg::msg::builder::Init_Ptu_header();
}

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__PTU__BUILDER_HPP_
