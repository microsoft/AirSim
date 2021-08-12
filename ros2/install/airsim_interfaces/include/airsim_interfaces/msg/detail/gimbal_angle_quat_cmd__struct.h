// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__STRUCT_H_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

// Struct defined in msg/GimbalAngleQuatCmd in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__GimbalAngleQuatCmd
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String camera_name;
  rosidl_runtime_c__String vehicle_name;
  geometry_msgs__msg__Quaternion orientation;
} airsim_interfaces__msg__GimbalAngleQuatCmd;

// Struct for a sequence of airsim_interfaces__msg__GimbalAngleQuatCmd.
typedef struct airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence
{
  airsim_interfaces__msg__GimbalAngleQuatCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_QUAT_CMD__STRUCT_H_
