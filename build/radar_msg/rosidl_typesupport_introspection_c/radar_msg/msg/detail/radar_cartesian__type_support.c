// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_msg/msg/detail/radar_cartesian__rosidl_typesupport_introspection_c.h"
#include "radar_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_msg/msg/detail/radar_cartesian__functions.h"
#include "radar_msg/msg/detail/radar_cartesian__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `stamps`
#include "builtin_interfaces/msg/time.h"
// Member `stamps`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `x`
// Member `y`
// Member `z`
// Member `gps_e`
// Member `gps_n`
// Member `gps_alt`
// Member `gps_qx`
// Member `gps_qy`
// Member `gps_qz`
// Member `gps_qw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `gps_frame`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_msg__msg__RadarCartesian__init(message_memory);
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_fini_function(void * message_memory)
{
  radar_msg__msg__RadarCartesian__fini(message_memory);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__stamps(
  const void * untyped_member)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__stamps(
  const void * untyped_member, size_t index)
{
  const builtin_interfaces__msg__Time__Sequence * member =
    (const builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__stamps(
  void * untyped_member, size_t index)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__stamps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const builtin_interfaces__msg__Time * item =
    ((const builtin_interfaces__msg__Time *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__stamps(untyped_member, index));
  builtin_interfaces__msg__Time * value =
    (builtin_interfaces__msg__Time *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__stamps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  builtin_interfaces__msg__Time * item =
    ((builtin_interfaces__msg__Time *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__stamps(untyped_member, index));
  const builtin_interfaces__msg__Time * value =
    (const builtin_interfaces__msg__Time *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__stamps(
  void * untyped_member, size_t size)
{
  builtin_interfaces__msg__Time__Sequence * member =
    (builtin_interfaces__msg__Time__Sequence *)(untyped_member);
  builtin_interfaces__msg__Time__Sequence__fini(member);
  return builtin_interfaces__msg__Time__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__x(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__y(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__z(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__z(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__z(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_e(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_e(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_e(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_e(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_e(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_e(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_e(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_e(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_n(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_n(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_n(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_n(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_n(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_n(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_n(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_n(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_alt(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_alt(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_alt(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_alt(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_alt(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_alt(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_alt(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_alt(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qx(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qx(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qx(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qx(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qx(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qx(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qx(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qx(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qy(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qy(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qy(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qy(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qy(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qy(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qy(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qy(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qz(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qz(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qz(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qz(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qz(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qz(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qz(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qz(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qw(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qw(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qw(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qw(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qw(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qw(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_member_array[14] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, stamps),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__stamps,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__stamps,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__stamps,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__stamps,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__stamps,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__stamps  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, x),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__x,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__x,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__x,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__x,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__x,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__x  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, y),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__y,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__y,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__y,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__y,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__y,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__y  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, z),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__z,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__z,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__z,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__z,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__z,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__z  // resize(index) function pointer
  },
  {
    "gps_e",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_e),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_e,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_e,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_e,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_e,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_e,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_e  // resize(index) function pointer
  },
  {
    "gps_n",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_n),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_n,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_n,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_n,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_n,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_n,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_n  // resize(index) function pointer
  },
  {
    "gps_alt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_alt),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_alt,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_alt,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_alt,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_alt,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_alt,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_alt  // resize(index) function pointer
  },
  {
    "gps_qx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_qx),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qx,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qx,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qx,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qx,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qx,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qx  // resize(index) function pointer
  },
  {
    "gps_qy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_qy),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qy,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qy,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qy,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qy,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qy,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qy  // resize(index) function pointer
  },
  {
    "gps_qz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_qz),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qz,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qz,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qz,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qz,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qz,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qz  // resize(index) function pointer
  },
  {
    "gps_qw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_qw),  // bytes offset in struct
    NULL,  // default value
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__size_function__RadarCartesian__gps_qw,  // size() function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_const_function__RadarCartesian__gps_qw,  // get_const(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__get_function__RadarCartesian__gps_qw,  // get(index) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__fetch_function__RadarCartesian__gps_qw,  // fetch(index, &value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__assign_function__RadarCartesian__gps_qw,  // assign(index, value) function pointer
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__resize_function__RadarCartesian__gps_qw  // resize(index) function pointer
  },
  {
    "gps_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, gps_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_pose_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msg__msg__RadarCartesian, robot_pose_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_members = {
  "radar_msg__msg",  // message namespace
  "RadarCartesian",  // message name
  14,  // number of fields
  sizeof(radar_msg__msg__RadarCartesian),
  radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_member_array,  // message members
  radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_type_support_handle = {
  0,
  &radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_msg, msg, RadarCartesian)() {
  radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_type_support_handle.typesupport_identifier) {
    radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_msg__msg__RadarCartesian__rosidl_typesupport_introspection_c__RadarCartesian_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
