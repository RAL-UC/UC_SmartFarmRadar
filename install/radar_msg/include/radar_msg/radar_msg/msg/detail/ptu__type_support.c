// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_msg:msg/Ptu.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_msg/msg/detail/ptu__rosidl_typesupport_introspection_c.h"
#include "radar_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_msg/msg/detail/ptu__functions.h"
#include "radar_msg/msg/detail/ptu__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_msg__msg__Ptu__init(message_memory);
}

void radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_fini_function(void * message_memory)
{
  radar_msg__msg__Ptu__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__Ptu, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pan_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__Ptu, pan_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tilt_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__Ptu, tilt_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_members = {
  "radar_msg__msg",  // message namespace
  "Ptu",  // message name
  3,  // number of fields
  sizeof(radar_msg__msg__Ptu),
  radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_member_array,  // message members
  radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_type_support_handle = {
  0,
  &radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_msg, msg, Ptu)() {
  radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_type_support_handle.typesupport_identifier) {
    radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_msg__msg__Ptu__rosidl_typesupport_introspection_c__Ptu_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
