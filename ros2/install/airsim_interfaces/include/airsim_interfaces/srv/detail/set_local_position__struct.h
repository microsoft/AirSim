// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:srv/SetLocalPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vehicle_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetLocalPosition in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__SetLocalPosition_Request
{
  double x;
  double y;
  double z;
  double yaw;
  rosidl_runtime_c__String vehicle_name;
} airsim_interfaces__srv__SetLocalPosition_Request;

// Struct for a sequence of airsim_interfaces__srv__SetLocalPosition_Request.
typedef struct airsim_interfaces__srv__SetLocalPosition_Request__Sequence
{
  airsim_interfaces__srv__SetLocalPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__SetLocalPosition_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetLocalPosition in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__SetLocalPosition_Response
{
  bool success;
  rosidl_runtime_c__String message;
} airsim_interfaces__srv__SetLocalPosition_Response;

// Struct for a sequence of airsim_interfaces__srv__SetLocalPosition_Response.
typedef struct airsim_interfaces__srv__SetLocalPosition_Response__Sequence
{
  airsim_interfaces__srv__SetLocalPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__SetLocalPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_H_
