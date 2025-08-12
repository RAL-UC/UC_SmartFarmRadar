// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_DATA__TRAITS_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_msg/msg/detail/radar_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace radar_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rows
  {
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << ", ";
  }

  // member: cols
  {
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << ", ";
  }

  // member: dtype
  {
    out << "dtype: ";
    rosidl_generator_traits::value_to_yaml(msg.dtype, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadarData & msg,
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

  // member: rows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << "\n";
  }

  // member: cols
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << "\n";
  }

  // member: dtype
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dtype: ";
    rosidl_generator_traits::value_to_yaml(msg.dtype, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarData & msg, bool use_flow_style = false)
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
  const radar_msg::msg::RadarData & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::msg::RadarData & msg)
{
  return radar_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::msg::RadarData>()
{
  return "radar_msg::msg::RadarData";
}

template<>
inline const char * name<radar_msg::msg::RadarData>()
{
  return "radar_msg/msg/RadarData";
}

template<>
struct has_fixed_size<radar_msg::msg::RadarData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<radar_msg::msg::RadarData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<radar_msg::msg::RadarData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_DATA__TRAITS_HPP_
