// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

// Struct defined in msg/VelCmd in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__VelCmd
{
  geometry_msgs__msg__Twist twist;
} airsim_interfaces__msg__VelCmd;

// Struct for a sequence of airsim_interfaces__msg__VelCmd.
typedef struct airsim_interfaces__msg__VelCmd__Sequence
{
  airsim_interfaces__msg__VelCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__VelCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_
