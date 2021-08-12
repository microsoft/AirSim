// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/Land in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__Land_Request
{
  bool wait_on_last_task;
} airsim_interfaces__srv__Land_Request;

// Struct for a sequence of airsim_interfaces__srv__Land_Request.
typedef struct airsim_interfaces__srv__Land_Request__Sequence
{
  airsim_interfaces__srv__Land_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__Land_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/Land in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__Land_Response
{
  bool success;
} airsim_interfaces__srv__Land_Response;

// Struct for a sequence of airsim_interfaces__srv__Land_Response.
typedef struct airsim_interfaces__srv__Land_Response__Sequence
{
  airsim_interfaces__srv__Land_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__Land_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND__STRUCT_H_
