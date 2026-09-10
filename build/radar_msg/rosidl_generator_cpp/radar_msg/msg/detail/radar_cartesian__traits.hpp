// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__TRAITS_HPP_
#define RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_msg/msg/detail/radar_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'stamps'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace radar_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarCartesian & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: stamps
  {
    if (msg.stamps.size() == 0) {
      out << "stamps: []";
    } else {
      out << "stamps: [";
      size_t pending_items = msg.stamps.size();
      for (auto item : msg.stamps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: x
  {
    if (msg.x.size() == 0) {
      out << "x: []";
    } else {
      out << "x: [";
      size_t pending_items = msg.x.size();
      for (auto item : msg.x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y
  {
    if (msg.y.size() == 0) {
      out << "y: []";
    } else {
      out << "y: [";
      size_t pending_items = msg.y.size();
      for (auto item : msg.y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: z
  {
    if (msg.z.size() == 0) {
      out << "z: []";
    } else {
      out << "z: [";
      size_t pending_items = msg.z.size();
      for (auto item : msg.z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_e
  {
    if (msg.gps_e.size() == 0) {
      out << "gps_e: []";
    } else {
      out << "gps_e: [";
      size_t pending_items = msg.gps_e.size();
      for (auto item : msg.gps_e) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_n
  {
    if (msg.gps_n.size() == 0) {
      out << "gps_n: []";
    } else {
      out << "gps_n: [";
      size_t pending_items = msg.gps_n.size();
      for (auto item : msg.gps_n) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_alt
  {
    if (msg.gps_alt.size() == 0) {
      out << "gps_alt: []";
    } else {
      out << "gps_alt: [";
      size_t pending_items = msg.gps_alt.size();
      for (auto item : msg.gps_alt) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_qx
  {
    if (msg.gps_qx.size() == 0) {
      out << "gps_qx: []";
    } else {
      out << "gps_qx: [";
      size_t pending_items = msg.gps_qx.size();
      for (auto item : msg.gps_qx) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_qy
  {
    if (msg.gps_qy.size() == 0) {
      out << "gps_qy: []";
    } else {
      out << "gps_qy: [";
      size_t pending_items = msg.gps_qy.size();
      for (auto item : msg.gps_qy) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_qz
  {
    if (msg.gps_qz.size() == 0) {
      out << "gps_qz: []";
    } else {
      out << "gps_qz: [";
      size_t pending_items = msg.gps_qz.size();
      for (auto item : msg.gps_qz) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_qw
  {
    if (msg.gps_qw.size() == 0) {
      out << "gps_qw: []";
    } else {
      out << "gps_qw: [";
      size_t pending_items = msg.gps_qw.size();
      for (auto item : msg.gps_qw) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_frame
  {
    out << "gps_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_frame, out);
    out << ", ";
  }

  // member: robot_pose_id
  {
    out << "robot_pose_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_pose_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadarCartesian & msg,
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

  // member: stamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.stamps.size() == 0) {
      out << "stamps: []\n";
    } else {
      out << "stamps:\n";
      for (auto item : msg.stamps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x.size() == 0) {
      out << "x: []\n";
    } else {
      out << "x:\n";
      for (auto item : msg.x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y.size() == 0) {
      out << "y: []\n";
    } else {
      out << "y:\n";
      for (auto item : msg.y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z.size() == 0) {
      out << "z: []\n";
    } else {
      out << "z:\n";
      for (auto item : msg.z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_e.size() == 0) {
      out << "gps_e: []\n";
    } else {
      out << "gps_e:\n";
      for (auto item : msg.gps_e) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_n.size() == 0) {
      out << "gps_n: []\n";
    } else {
      out << "gps_n:\n";
      for (auto item : msg.gps_n) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_alt.size() == 0) {
      out << "gps_alt: []\n";
    } else {
      out << "gps_alt:\n";
      for (auto item : msg.gps_alt) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_qx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_qx.size() == 0) {
      out << "gps_qx: []\n";
    } else {
      out << "gps_qx:\n";
      for (auto item : msg.gps_qx) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_qy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_qy.size() == 0) {
      out << "gps_qy: []\n";
    } else {
      out << "gps_qy:\n";
      for (auto item : msg.gps_qy) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_qz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_qz.size() == 0) {
      out << "gps_qz: []\n";
    } else {
      out << "gps_qz:\n";
      for (auto item : msg.gps_qz) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_qw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_qw.size() == 0) {
      out << "gps_qw: []\n";
    } else {
      out << "gps_qw:\n";
      for (auto item : msg.gps_qw) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_frame, out);
    out << "\n";
  }

  // member: robot_pose_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_pose_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_pose_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarCartesian & msg, bool use_flow_style = false)
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
  const radar_msg::msg::RadarCartesian & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::msg::RadarCartesian & msg)
{
  return radar_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::msg::RadarCartesian>()
{
  return "radar_msg::msg::RadarCartesian";
}

template<>
inline const char * name<radar_msg::msg::RadarCartesian>()
{
  return "radar_msg/msg/RadarCartesian";
}

template<>
struct has_fixed_size<radar_msg::msg::RadarCartesian>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<radar_msg::msg::RadarCartesian>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<radar_msg::msg::RadarCartesian>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__TRAITS_HPP_
