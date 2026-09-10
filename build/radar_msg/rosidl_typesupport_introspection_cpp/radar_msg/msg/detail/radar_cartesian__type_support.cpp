// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "radar_msg/msg/detail/radar_cartesian__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace radar_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RadarCartesian_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) radar_msg::msg::RadarCartesian(_init);
}

void RadarCartesian_fini_function(void * message_memory)
{
  auto typed_message = static_cast<radar_msg::msg::RadarCartesian *>(message_memory);
  typed_message->~RadarCartesian();
}

size_t size_function__RadarCartesian__stamps(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<builtin_interfaces::msg::Time> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__stamps(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<builtin_interfaces::msg::Time> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__stamps(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<builtin_interfaces::msg::Time> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__stamps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const builtin_interfaces::msg::Time *>(
    get_const_function__RadarCartesian__stamps(untyped_member, index));
  auto & value = *reinterpret_cast<builtin_interfaces::msg::Time *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__stamps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<builtin_interfaces::msg::Time *>(
    get_function__RadarCartesian__stamps(untyped_member, index));
  const auto & value = *reinterpret_cast<const builtin_interfaces::msg::Time *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__stamps(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<builtin_interfaces::msg::Time> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__z(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__z(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__z(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__z(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__z(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__z(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_e(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_e(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_e(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_e(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_e(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_e(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_n(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_n(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_n(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_n(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_n(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_n(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_n(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_n(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_alt(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_alt(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_alt(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_alt(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_alt(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_alt(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_alt(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_alt(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_qx(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_qx(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_qx(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_qx(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_qx(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_qx(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_qx(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_qx(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_qy(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_qy(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_qy(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_qy(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_qy(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_qy(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_qy(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_qy(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_qz(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_qz(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_qz(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_qz(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_qz(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_qz(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_qz(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_qz(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RadarCartesian__gps_qw(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarCartesian__gps_qw(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarCartesian__gps_qw(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarCartesian__gps_qw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarCartesian__gps_qw(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarCartesian__gps_qw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarCartesian__gps_qw(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__RadarCartesian__gps_qw(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RadarCartesian_message_member_array[14] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, stamps),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__stamps,  // size() function pointer
    get_const_function__RadarCartesian__stamps,  // get_const(index) function pointer
    get_function__RadarCartesian__stamps,  // get(index) function pointer
    fetch_function__RadarCartesian__stamps,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__stamps,  // assign(index, value) function pointer
    resize_function__RadarCartesian__stamps  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, x),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__x,  // size() function pointer
    get_const_function__RadarCartesian__x,  // get_const(index) function pointer
    get_function__RadarCartesian__x,  // get(index) function pointer
    fetch_function__RadarCartesian__x,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__x,  // assign(index, value) function pointer
    resize_function__RadarCartesian__x  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, y),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__y,  // size() function pointer
    get_const_function__RadarCartesian__y,  // get_const(index) function pointer
    get_function__RadarCartesian__y,  // get(index) function pointer
    fetch_function__RadarCartesian__y,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__y,  // assign(index, value) function pointer
    resize_function__RadarCartesian__y  // resize(index) function pointer
  },
  {
    "z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, z),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__z,  // size() function pointer
    get_const_function__RadarCartesian__z,  // get_const(index) function pointer
    get_function__RadarCartesian__z,  // get(index) function pointer
    fetch_function__RadarCartesian__z,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__z,  // assign(index, value) function pointer
    resize_function__RadarCartesian__z  // resize(index) function pointer
  },
  {
    "gps_e",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_e),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_e,  // size() function pointer
    get_const_function__RadarCartesian__gps_e,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_e,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_e,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_e,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_e  // resize(index) function pointer
  },
  {
    "gps_n",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_n),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_n,  // size() function pointer
    get_const_function__RadarCartesian__gps_n,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_n,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_n,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_n,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_n  // resize(index) function pointer
  },
  {
    "gps_alt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_alt),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_alt,  // size() function pointer
    get_const_function__RadarCartesian__gps_alt,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_alt,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_alt,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_alt,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_alt  // resize(index) function pointer
  },
  {
    "gps_qx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_qx),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_qx,  // size() function pointer
    get_const_function__RadarCartesian__gps_qx,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_qx,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_qx,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_qx,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_qx  // resize(index) function pointer
  },
  {
    "gps_qy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_qy),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_qy,  // size() function pointer
    get_const_function__RadarCartesian__gps_qy,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_qy,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_qy,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_qy,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_qy  // resize(index) function pointer
  },
  {
    "gps_qz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_qz),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_qz,  // size() function pointer
    get_const_function__RadarCartesian__gps_qz,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_qz,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_qz,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_qz,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_qz  // resize(index) function pointer
  },
  {
    "gps_qw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_qw),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarCartesian__gps_qw,  // size() function pointer
    get_const_function__RadarCartesian__gps_qw,  // get_const(index) function pointer
    get_function__RadarCartesian__gps_qw,  // get(index) function pointer
    fetch_function__RadarCartesian__gps_qw,  // fetch(index, &value) function pointer
    assign_function__RadarCartesian__gps_qw,  // assign(index, value) function pointer
    resize_function__RadarCartesian__gps_qw  // resize(index) function pointer
  },
  {
    "gps_frame",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, gps_frame),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_pose_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg::msg::RadarCartesian, robot_pose_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RadarCartesian_message_members = {
  "radar_msg::msg",  // message namespace
  "RadarCartesian",  // message name
  14,  // number of fields
  sizeof(radar_msg::msg::RadarCartesian),
  RadarCartesian_message_member_array,  // message members
  RadarCartesian_init_function,  // function to initialize message memory (memory has to be allocated)
  RadarCartesian_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RadarCartesian_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RadarCartesian_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace radar_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msg::msg::RadarCartesian>()
{
  return &::radar_msg::msg::rosidl_typesupport_introspection_cpp::RadarCartesian_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msg, msg, RadarCartesian)() {
  return &::radar_msg::msg::rosidl_typesupport_introspection_cpp::RadarCartesian_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
