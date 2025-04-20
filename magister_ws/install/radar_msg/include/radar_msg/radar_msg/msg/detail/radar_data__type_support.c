// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_msg:msg/RadarData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_msg/msg/detail/radar_data__rosidl_typesupport_introspection_c.h"
#include "radar_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_msg/msg/detail/radar_data__functions.h"
#include "radar_msg/msg/detail/radar_data__struct.h"


// Include directives for member types
// Member `dtype`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_msg__msg__RadarData__init(message_memory);
}

void radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_fini_function(void * message_memory)
{
  radar_msg__msg__RadarData__fini(message_memory);
}

size_t radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__size_function__RadarData__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_const_function__RadarData__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_function__RadarData__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__fetch_function__RadarData__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_const_function__RadarData__data(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__assign_function__RadarData__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_function__RadarData__data(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__resize_function__RadarData__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_member_array[4] = {
  {
    "rows",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarData, rows),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cols",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarData, cols),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dtype",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarData, dtype),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarData, data),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__size_function__RadarData__data,  // size() function pointer
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_const_function__RadarData__data,  // get_const(index) function pointer
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__get_function__RadarData__data,  // get(index) function pointer
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__fetch_function__RadarData__data,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__assign_function__RadarData__data,  // assign(index, value) function pointer
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__resize_function__RadarData__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_members = {
  "radar_msg__msg",  // message namespace
  "RadarData",  // message name
  4,  // number of fields
  sizeof(radar_msg__msg__RadarData),
  radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_member_array,  // message members
  radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_type_support_handle = {
  0,
  &radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_msg, msg, RadarData)() {
  if (!radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_type_support_handle.typesupport_identifier) {
    radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_msg__msg__RadarData__rosidl_typesupport_introspection_c__RadarData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
