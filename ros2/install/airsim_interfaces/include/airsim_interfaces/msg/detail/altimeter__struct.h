// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_

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

// Struct defined in msg/Altimeter in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__Altimeter
{
  std_msgs__msg__Header header;
  float altitude;
  float pressure;
  float qnh;
} airsim_interfaces__msg__Altimeter;

// Struct for a sequence of airsim_interfaces__msg__Altimeter.
typedef struct airsim_interfaces__msg__Altimeter__Sequence
{
  airsim_interfaces__msg__Altimeter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__Altimeter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_
