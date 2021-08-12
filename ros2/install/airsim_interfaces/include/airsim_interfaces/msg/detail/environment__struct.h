// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_H_

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
// Member 'position'
// Member 'gravity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'geo_point'
#include "geographic_msgs/msg/detail/geo_point__struct.h"

// Struct defined in msg/Environment in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__Environment
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 position;
  geographic_msgs__msg__GeoPoint geo_point;
  geometry_msgs__msg__Vector3 gravity;
  float air_pressure;
  float temperature;
  float air_density;
} airsim_interfaces__msg__Environment;

// Struct for a sequence of airsim_interfaces__msg__Environment.
typedef struct airsim_interfaces__msg__Environment__Sequence
{
  airsim_interfaces__msg__Environment * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__Environment__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_H_
