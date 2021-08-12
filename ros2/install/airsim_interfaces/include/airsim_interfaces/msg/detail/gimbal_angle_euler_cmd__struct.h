// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'camera_name'
// Member 'vehicle_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/GimbalAngleEulerCmd in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__GimbalAngleEulerCmd
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String camera_name;
  rosidl_runtime_c__String vehicle_name;
  double roll;
  double pitch;
  double yaw;
} airsim_interfaces__msg__GimbalAngleEulerCmd;

// Struct for a sequence of airsim_interfaces__msg__GimbalAngleEulerCmd.
typedef struct airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence
{
  airsim_interfaces__msg__GimbalAngleEulerCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_H_
