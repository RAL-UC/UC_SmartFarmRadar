// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_DATA__BUILDER_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msg/msg/detail/radar_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msg
{

namespace msg
{

namespace builder
{

class Init_RadarData_data
{
public:
  explicit Init_RadarData_data(::radar_msg::msg::RadarData & msg)
  : msg_(msg)
  {}
  ::radar_msg::msg::RadarData data(::radar_msg::msg::RadarData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msg::msg::RadarData msg_;
};

class Init_RadarData_dtype
{
public:
  explicit Init_RadarData_dtype(::radar_msg::msg::RadarData & msg)
  : msg_(msg)
  {}
  Init_RadarData_data dtype(::radar_msg::msg::RadarData::_dtype_type arg)
  {
    msg_.dtype = std::move(arg);
    return Init_RadarData_data(msg_);
  }

private:
  ::radar_msg::msg::RadarData msg_;
};

class Init_RadarData_cols
{
public:
  explicit Init_RadarData_cols(::radar_msg::msg::RadarData & msg)
  : msg_(msg)
  {}
  Init_RadarData_dtype cols(::radar_msg::msg::RadarData::_cols_type arg)
  {
    msg_.cols = std::move(arg);
    return Init_RadarData_dtype(msg_);
  }

private:
  ::radar_msg::msg::RadarData msg_;
};

class Init_RadarData_rows
{
public:
  explicit Init_RadarData_rows(::radar_msg::msg::RadarData & msg)
  : msg_(msg)
  {}
  Init_RadarData_cols rows(::radar_msg::msg::RadarData::_rows_type arg)
  {
    msg_.rows = std::move(arg);
    return Init_RadarData_cols(msg_);
  }

private:
  ::radar_msg::msg::RadarData msg_;
};

class Init_RadarData_header
{
public:
  Init_RadarData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarData_rows header(::radar_msg::msg::RadarData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarData_rows(msg_);
  }

private:
  ::radar_msg::msg::RadarData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msg::msg::RadarData>()
{
  return radar_msg::msg::builder::Init_RadarData_header();
}

}  // namespace radar_msg

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_DATA__BUILDER_HPP_
