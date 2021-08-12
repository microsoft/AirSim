// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/GPSYaw in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__GPSYaw
{
  double latitude;
  double longitude;
  double altitude;
  double yaw;
} airsim_interfaces__msg__GPSYaw;

// Struct for a sequence of airsim_interfaces__msg__GPSYaw.
typedef struct airsim_interfaces__msg__GPSYaw__Sequence
{
  airsim_interfaces__msg__GPSYaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__GPSYaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__STRUCT_H_
