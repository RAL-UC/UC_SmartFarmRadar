// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_msg:action/PtuSweep.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__TRAITS_HPP_
#define RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_msg/action/detail/ptu_sweep__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_ptu'
#include "radar_msg/msg/detail/ptu__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_ptu
  {
    out << "target_ptu: ";
    to_flow_style_yaml(msg.target_ptu, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_ptu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_ptu:\n";
    to_block_style_yaml(msg.target_ptu, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_Goal & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_Goal>()
{
  return "radar_msg::action::PtuSweep_Goal";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_Goal>()
{
  return "radar_msg/action/PtuSweep_Goal";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_Goal>
  : std::integral_constant<bool, has_fixed_size<radar_msg::msg::Ptu>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_Goal>
  : std::integral_constant<bool, has_bounded_size<radar_msg::msg::Ptu>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_Result & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_Result>()
{
  return "radar_msg::action::PtuSweep_Result";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_Result>()
{
  return "radar_msg/action/PtuSweep_Result";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<radar_msg::action::PtuSweep_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_pan_deg
  {
    out << "current_pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.current_pan_deg, out);
    out << ", ";
  }

  // member: current_tilt_deg
  {
    out << "current_tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tilt_deg, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_pan_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.current_pan_deg, out);
    out << "\n";
  }

  // member: current_tilt_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tilt_deg, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_Feedback & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_Feedback>()
{
  return "radar_msg::action::PtuSweep_Feedback";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_Feedback>()
{
  return "radar_msg/action/PtuSweep_Feedback";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<radar_msg::action::PtuSweep_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "radar_msg/action/detail/ptu_sweep__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_SendGoal_Request & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_SendGoal_Request>()
{
  return "radar_msg::action::PtuSweep_SendGoal_Request";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_SendGoal_Request>()
{
  return "radar_msg/action/PtuSweep_SendGoal_Request";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<radar_msg::action::PtuSweep_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<radar_msg::action::PtuSweep_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_SendGoal_Response & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_SendGoal_Response>()
{
  return "radar_msg::action::PtuSweep_SendGoal_Response";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_SendGoal_Response>()
{
  return "radar_msg/action/PtuSweep_SendGoal_Response";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<radar_msg::action::PtuSweep_SendGoal>()
{
  return "radar_msg::action::PtuSweep_SendGoal";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_SendGoal>()
{
  return "radar_msg/action/PtuSweep_SendGoal";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<radar_msg::action::PtuSweep_SendGoal_Request>::value &&
    has_fixed_size<radar_msg::action::PtuSweep_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<radar_msg::action::PtuSweep_SendGoal_Request>::value &&
    has_bounded_size<radar_msg::action::PtuSweep_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<radar_msg::action::PtuSweep_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<radar_msg::action::PtuSweep_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<radar_msg::action::PtuSweep_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_GetResult_Request & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_GetResult_Request>()
{
  return "radar_msg::action::PtuSweep_GetResult_Request";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_GetResult_Request>()
{
  return "radar_msg/action/PtuSweep_GetResult_Request";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_GetResult_Response & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_GetResult_Response>()
{
  return "radar_msg::action::PtuSweep_GetResult_Response";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_GetResult_Response>()
{
  return "radar_msg/action/PtuSweep_GetResult_Response";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<radar_msg::action::PtuSweep_Result>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<radar_msg::action::PtuSweep_Result>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<radar_msg::action::PtuSweep_GetResult>()
{
  return "radar_msg::action::PtuSweep_GetResult";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_GetResult>()
{
  return "radar_msg/action/PtuSweep_GetResult";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<radar_msg::action::PtuSweep_GetResult_Request>::value &&
    has_fixed_size<radar_msg::action::PtuSweep_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<radar_msg::action::PtuSweep_GetResult_Request>::value &&
    has_bounded_size<radar_msg::action::PtuSweep_GetResult_Response>::value
  >
{
};

template<>
struct is_service<radar_msg::action::PtuSweep_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<radar_msg::action::PtuSweep_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<radar_msg::action::PtuSweep_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "radar_msg/action/detail/ptu_sweep__traits.hpp"

namespace radar_msg
{

namespace action
{

inline void to_flow_style_yaml(
  const PtuSweep_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PtuSweep_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PtuSweep_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace radar_msg

namespace rosidl_generator_traits
{

[[deprecated("use radar_msg::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msg::action::PtuSweep_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msg::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msg::action::to_yaml() instead")]]
inline std::string to_yaml(const radar_msg::action::PtuSweep_FeedbackMessage & msg)
{
  return radar_msg::action::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msg::action::PtuSweep_FeedbackMessage>()
{
  return "radar_msg::action::PtuSweep_FeedbackMessage";
}

template<>
inline const char * name<radar_msg::action::PtuSweep_FeedbackMessage>()
{
  return "radar_msg/action/PtuSweep_FeedbackMessage";
}

template<>
struct has_fixed_size<radar_msg::action::PtuSweep_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<radar_msg::action::PtuSweep_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<radar_msg::action::PtuSweep_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<radar_msg::action::PtuSweep_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<radar_msg::action::PtuSweep_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<radar_msg::action::PtuSweep>
  : std::true_type
{
};

template<>
struct is_action_goal<radar_msg::action::PtuSweep_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<radar_msg::action::PtuSweep_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<radar_msg::action::PtuSweep_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // RADAR_MSG__ACTION__DETAIL__PTU_SWEEP__TRAITS_HPP_
