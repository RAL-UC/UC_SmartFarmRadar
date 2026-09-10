// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_msg:msg/Ptu.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__PTU__TRAITS_HPP_
#define RADAR_MSG__MSG__DETAIL__PTU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_msg/msg/detail/ptu__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace radar_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Ptu & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pan_deg
  {
    out << "pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_deg, out);
    out << ", ";
  }

  // member: tilt_deg
  {
    out << "tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Ptu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: pan_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_deg, out);
    out << "\n";
  }

  // member: tilt_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Ptu & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::msg::Ptu & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::msg::Ptu & msg)
{
  return radar_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::msg::Ptu>()
{
  return "radar_msg::msg::Ptu";
}

template<>
inline const char * name<radar_msg::msg::Ptu>()
{
  return "radar_msg/msg/Ptu";
}

template<>
struct has_fixed_size<radar_msg::msg::Ptu>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<radar_msg::msg::Ptu>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<radar_msg::msg::Ptu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_MSG__MSG__DETAIL__PTU__TRAITS_HPP_
