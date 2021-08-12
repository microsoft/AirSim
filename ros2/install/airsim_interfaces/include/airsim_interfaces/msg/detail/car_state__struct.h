// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.h"

// Struct defined in msg/CarState in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__CarState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__PoseWithCovariance pose;
  geometry_msgs__msg__TwistWithCovariance twist;
  float speed;
  int8_t gear;
  float rpm;
  float maxrpm;
  bool handbrake;
} airsim_interfaces__msg__CarState;

// Struct for a sequence of airsim_interfaces__msg__CarState.
typedef struct airsim_interfaces__msg__CarState__Sequence
{
  airsim_interfaces__msg__CarState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__CarState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_H_
