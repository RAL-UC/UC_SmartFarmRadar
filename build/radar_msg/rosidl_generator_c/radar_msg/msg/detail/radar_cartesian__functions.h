// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from radar_msg:msg/RadarCartesian.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__FUNCTIONS_H_
#define RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "radar_msg/msg/rosidl_generator_c__visibility_control.h"

#include "radar_msg/msg/detail/radar_cartesian__struct.h"

/// Initialize msg/RadarCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * radar_msg__msg__RadarCartesian
 * )) before or use
 * radar_msg__msg__RadarCartesian__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__init(radar_msg__msg__RadarCartesian * msg);

/// Finalize msg/RadarCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
void
radar_msg__msg__RadarCartesian__fini(radar_msg__msg__RadarCartesian * msg);

/// Create msg/RadarCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * radar_msg__msg__RadarCartesian__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
radar_msg__msg__RadarCartesian *
radar_msg__msg__RadarCartesian__create();

/// Destroy msg/RadarCartesian message.
/**
 * It calls
 * radar_msg__msg__RadarCartesian__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
void
radar_msg__msg__RadarCartesian__destroy(radar_msg__msg__RadarCartesian * msg);

/// Check for msg/RadarCartesian message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__are_equal(const radar_msg__msg__RadarCartesian * lhs, const radar_msg__msg__RadarCartesian * rhs);

/// Copy a msg/RadarCartesian message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__copy(
  const radar_msg__msg__RadarCartesian * input,
  radar_msg__msg__RadarCartesian * output);

/// Initialize array of msg/RadarCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * radar_msg__msg__RadarCartesian__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__Sequence__init(radar_msg__msg__RadarCartesian__Sequence * array, size_t size);

/// Finalize array of msg/RadarCartesian messages.
/**
 * It calls
 * radar_msg__msg__RadarCartesian__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
void
radar_msg__msg__RadarCartesian__Sequence__fini(radar_msg__msg__RadarCartesian__Sequence * array);

/// Create array of msg/RadarCartesian messages.
/**
 * It allocates the memory for the array and calls
 * radar_msg__msg__RadarCartesian__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
radar_msg__msg__RadarCartesian__Sequence *
radar_msg__msg__RadarCartesian__Sequence__create(size_t size);

/// Destroy array of msg/RadarCartesian messages.
/**
 * It calls
 * radar_msg__msg__RadarCartesian__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
void
radar_msg__msg__RadarCartesian__Sequence__destroy(radar_msg__msg__RadarCartesian__Sequence * array);

/// Check for msg/RadarCartesian message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__Sequence__are_equal(const radar_msg__msg__RadarCartesian__Sequence * lhs, const radar_msg__msg__RadarCartesian__Sequence * rhs);

/// Copy an array of msg/RadarCartesian messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msg
bool
radar_msg__msg__RadarCartesian__Sequence__copy(
  const radar_msg__msg__RadarCartesian__Sequence * input,
  radar_msg__msg__RadarCartesian__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSG__MSG__DETAIL__RADAR_CARTESIAN__FUNCTIONS_H_
