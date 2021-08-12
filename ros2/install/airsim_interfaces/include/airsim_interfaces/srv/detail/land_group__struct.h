// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:srv/LandGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__STRUCT_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vehicle_names'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/LandGroup in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__LandGroup_Request
{
  rosidl_runtime_c__String__Sequence vehicle_names;
  bool wait_on_last_task;
} airsim_interfaces__srv__LandGroup_Request;

// Struct for a sequence of airsim_interfaces__srv__LandGroup_Request.
typedef struct airsim_interfaces__srv__LandGroup_Request__Sequence
{
  airsim_interfaces__srv__LandGroup_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__LandGroup_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/LandGroup in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__LandGroup_Response
{
  bool success;
} airsim_interfaces__srv__LandGroup_Response;

// Struct for a sequence of airsim_interfaces__srv__LandGroup_Response.
typedef struct airsim_interfaces__srv__LandGroup_Response__Sequence
{
  airsim_interfaces__srv__LandGroup_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__LandGroup_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__STRUCT_H_
