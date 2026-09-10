// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__BUILDER_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msg/msg/detail/radar_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msg
{

namespace msg
{

namespace builder
{

class Init_RadarCartesian_robot_pose_id
{
public:
  explicit Init_RadarCartesian_robot_pose_id(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  ::radar_msg::msg::RadarCartesian robot_pose_id(::radar_msg::msg::RadarCartesian::_robot_pose_id_type arg)
  {
    msg_.robot_pose_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_frame
{
public:
  explicit Init_RadarCartesian_gps_frame(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_robot_pose_id gps_frame(::radar_msg::msg::RadarCartesian::_gps_frame_type arg)
  {
    msg_.gps_frame = std::move(arg);
    return Init_RadarCartesian_robot_pose_id(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_qw
{
public:
  explicit Init_RadarCartesian_gps_qw(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_frame gps_qw(::radar_msg::msg::RadarCartesian::_gps_qw_type arg)
  {
    msg_.gps_qw = std::move(arg);
    return Init_RadarCartesian_gps_frame(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_qz
{
public:
  explicit Init_RadarCartesian_gps_qz(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_qw gps_qz(::radar_msg::msg::RadarCartesian::_gps_qz_type arg)
  {
    msg_.gps_qz = std::move(arg);
    return Init_RadarCartesian_gps_qw(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_qy
{
public:
  explicit Init_RadarCartesian_gps_qy(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_qz gps_qy(::radar_msg::msg::RadarCartesian::_gps_qy_type arg)
  {
    msg_.gps_qy = std::move(arg);
    return Init_RadarCartesian_gps_qz(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_qx
{
public:
  explicit Init_RadarCartesian_gps_qx(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_qy gps_qx(::radar_msg::msg::RadarCartesian::_gps_qx_type arg)
  {
    msg_.gps_qx = std::move(arg);
    return Init_RadarCartesian_gps_qy(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_alt
{
public:
  explicit Init_RadarCartesian_gps_alt(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_qx gps_alt(::radar_msg::msg::RadarCartesian::_gps_alt_type arg)
  {
    msg_.gps_alt = std::move(arg);
    return Init_RadarCartesian_gps_qx(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_n
{
public:
  explicit Init_RadarCartesian_gps_n(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_alt gps_n(::radar_msg::msg::RadarCartesian::_gps_n_type arg)
  {
    msg_.gps_n = std::move(arg);
    return Init_RadarCartesian_gps_alt(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_gps_e
{
public:
  explicit Init_RadarCartesian_gps_e(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_n gps_e(::radar_msg::msg::RadarCartesian::_gps_e_type arg)
  {
    msg_.gps_e = std::move(arg);
    return Init_RadarCartesian_gps_n(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_z
{
public:
  explicit Init_RadarCartesian_z(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_gps_e z(::radar_msg::msg::RadarCartesian::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_RadarCartesian_gps_e(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_y
{
public:
  explicit Init_RadarCartesian_y(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_z y(::radar_msg::msg::RadarCartesian::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RadarCartesian_z(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_x
{
public:
  explicit Init_RadarCartesian_x(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_y x(::radar_msg::msg::RadarCartesian::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RadarCartesian_y(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_stamps
{
public:
  explicit Init_RadarCartesian_stamps(::radar_msg::msg::RadarCartesian & msg)
  : msg_(msg)
  {}
  Init_RadarCartesian_x stamps(::radar_msg::msg::RadarCartesian::_stamps_type arg)
  {
    msg_.stamps = std::move(arg);
    return Init_RadarCartesian_x(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

class Init_RadarCartesian_header
{
public:
  Init_RadarCartesian_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarCartesian_stamps header(::radar_msg::msg::RadarCartesian::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarCartesian_stamps(msg_);
  }

private:
  ::radar_msg::msg::RadarCartesian msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::msg::RadarCartesian>()
{
  return radar_msg::msg::builder::Init_RadarCartesian_header();
}

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__BUILDER_HPP_
