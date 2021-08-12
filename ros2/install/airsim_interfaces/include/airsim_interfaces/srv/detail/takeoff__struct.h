// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:srv/Takeoff.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF__STRUCT_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/Takeoff in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__Takeoff_Request
{
  bool wait_on_last_task;
} airsim_interfaces__srv__Takeoff_Request;

// Struct for a sequence of airsim_interfaces__srv__Takeoff_Request.
typedef struct airsim_interfaces__srv__Takeoff_Request__Sequence
{
  airsim_interfaces__srv__Takeoff_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__Takeoff_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/Takeoff in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__Takeoff_Response
{
  bool success;
} airsim_interfaces__srv__Takeoff_Response;

// Struct for a sequence of airsim_interfaces__srv__Takeoff_Response.
typedef struct airsim_interfaces__srv__Takeoff_Response__Sequence
{
  airsim_interfaces__srv__Takeoff_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__Takeoff_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF__STRUCT_H_
