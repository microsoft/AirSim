// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/CarControls.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_H_

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

// Struct defined in msg/CarControls in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__CarControls
{
  std_msgs__msg__Header header;
  float throttle;
  float brake;
  float steering;
  bool handbrake;
  bool manual;
  int8_t manual_gear;
  bool gear_immediate;
} airsim_interfaces__msg__CarControls;

// Struct for a sequence of airsim_interfaces__msg__CarControls.
typedef struct airsim_interfaces__msg__CarControls__Sequence
{
  airsim_interfaces__msg__CarControls * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__CarControls__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__STRUCT_H_
